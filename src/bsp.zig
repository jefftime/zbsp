const std = @import("std");
const Map = @import("map.zig").Map;
const v3 = @import("geo.zig").v3;
const Brush = Map.Brush;
const Plane = @import("geo.zig").Plane;
const Mat3 = @import("math.zig").Mat3;
const streq = @import("util.zig").streq;

const pr = std.debug.print;
const ON_EPSILON: f32 = 0.1;
const DIST_EPSILON: f32 = 0.01;
const NORMAL_EPSILON: f32 = 1e-5;
const WINDING_EPSILON: f32 = 1e-3;
const VERTEX_EPSILON: f32 = 0.01;

pub const Bsp = struct {
    plane_id: u32 = 0,
    front: ?*Bsp = null,
    back: ?*Bsp = null,

    pub fn print_digraph(self: *const Bsp) void {
        _ = self;
        std.log.debug("Todo!", .{});
    }
};

const GeoData = struct {
    planes: []Plane,
    points: []v3,
    faces: []GeoFace,
};

const GeoFace = struct {
    u: v3,
    v: v3,
    plane_id: u32,
    brush_id: u32,
    point_ids: []u32,
    points: []v3,

    pub fn sort_points(self: *GeoFace) void {
        const PointComparator = struct {
            const Context = struct {
                u: v3,
                v: v3,
                centroid: v3,
                points: []v3,
            };

            fn less_than(ctx: Context, lhs: u32, rhs: u32) bool {
                const lhsx = ctx.u.dot(ctx.points[lhs]);
                const lhsy = ctx.v.dot(ctx.points[lhs]);
                const rhsx = ctx.u.dot(ctx.points[rhs]);
                const rhsy = ctx.v.dot(ctx.points[rhs]);
                const cx = ctx.u.dot(ctx.centroid);
                const cy = ctx.v.dot(ctx.centroid);

                const lhs_angle = std.math.atan2(lhsy - cy, lhsx - cx);
                const rhs_angle = std.math.atan2(rhsy - cy, rhsx - cx);

                return lhs_angle < rhs_angle;
            }
        };

        var centroid = self.points[self.point_ids[0]];
        for (self.point_ids[1..]) |pt_id| {
            centroid = centroid.add(self.points[pt_id]);
        }
        centroid = centroid.div(@floatFromInt(self.point_ids.len));

        std.sort.pdq(
            u32,
            self.point_ids,
            PointComparator.Context{
                .u = self.u,
                .v = self.v,
                .centroid = centroid,
                .points = self.points,
            },
            PointComparator.less_than,
        );
    }
};

fn planes_from_brushplanes(al: std.mem.Allocator, bps: []Map.Plane) ![]Plane {
    var planes = try std.ArrayListUnmanaged(Plane).initCapacity(al, bps.len);
    for (bps) |bp| {
        var plane = Plane.from_points(
            bp.a,
            bp.b,
            bp.c,
        ) orelse return error.InvalidPlane;
        plane.u = bp.u;
        plane.v = bp.v;

        try planes.append(al, plane);
    }

    return planes.items;
}

fn point_on_plane(pt: v3, plane: Plane) bool {
    return std.math.approxEqAbs(
        f32,
        plane.normal.dot(pt) + plane.offset,
        0.0,
        ON_EPSILON,
    );
}

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    for (planes, 0..) |plane, i| {
        _ = i;
        const pt_offset = plane.normal.dot(pt) + plane.offset;

        const on_plane = std.math.approxEqAbs(f32, pt_offset, 0.0, ON_EPSILON);
        // The plane halfspaces that construct a solid shape define the solid
        // portion as being 'in front of the plane' rather than behind
        const within_volume = pt_offset > 0.0;

        result = result and (within_volume or on_plane);
    }

    return result;
}

fn point_from_planes(a: Plane, b: Plane, c: Plane) ?v3 {
    const c1 = b.normal.cross(c.normal);
    const c2 = c.normal.cross(a.normal);
    const c3 = a.normal.cross(b.normal);

    const numerator =
        c1.mul(-a.offset).add(c2.mul(-b.offset)).add(c3.mul(-c.offset));
    const denominator = a.normal.dot(c1);

    if (std.math.approxEqAbs(f32, denominator, 0.0, ON_EPSILON)) {
        return null;
    }

    return numerator.div(denominator);
}

fn points_to_obj(name: []const u8, points: []v3, faces: []u32) void {
    pr("o {s}\n", .{name});
    for (points) |pt| pr("v {d:.1} {d:.1} {d:.1}\n", .{ pt.x, pt.y, pt.z });
    var i: u32 = 0;
    while (i < faces.len) : (i += 3) {
        // Obj face indices start at 1
        pr("f {} {} {}\n", .{
            faces[i] + 1,
            faces[i + 1] + 1,
            faces[i + 2] + 1,
        });
    }
}

fn dedupe_points(points: *[]v3, faces: []GeoFace) void {
    if (points.len == 0) return;

    // Index of the last element
    var end = points.len - 1;

    for (0..end) |i| {
        var j = i + 1;
        while (j < end + 1) : (j += 1) {
            if (!points.*[i].eq(points.*[j], VERTEX_EPSILON)) continue;

            // TODO: Probably put all the point replacements in a HashMap and
            // iterate over faces a single time
            for (faces) |*face| {
                for (face.*.point_ids) |*pt_id| {
                    if (pt_id.* == @as(u32, @intCast(j))) {
                        pt_id.* = @as(u32, @intCast(i));
                    }
                    if (pt_id.* == @as(u32, @intCast(end))) {
                        pt_id.* = @as(u32, @intCast(j));
                    }
                }
            }

            if (j == end) {
                end -= 1;
                continue;
            }

            const tmp = points.*[j];
            points.*[j] = points.*[end];
            points.*[end] = tmp;

            end -= 1;
            j -= 1;
        }
    }

    points.* = points.*[0 .. end + 1];
}

fn triangulate(al: std.mem.Allocator, data: *GeoData) !void {
    var newfaces = try std.ArrayListUnmanaged(GeoFace).initCapacity(
        al,
        data.faces.len * 2,
    );

    var plane_pts = try std.ArrayListUnmanaged(u32).initCapacity(al, 128);
    for (0..data.planes.len) |plane_id| {
        // TODO: If this is a bottleneck we should look at using a
        // dictionary to store plane-point relationships somehow
        plane_pts.clearRetainingCapacity();
        for (data.points, 0..) |pt, i| {
            if (!point_on_plane(pt, data.planes[plane_id])) continue;
            try plane_pts.append(al, @intCast(i));
        }

        if (plane_pts.items.len == 3) {
            const normal = v3.from_points(&[_]v3{
                data.points[plane_pts.items[0]],
                data.points[plane_pts.items[1]],
                data.points[plane_pts.items[2]],
            }) orelse return error.TriangulateInvalidPlanePoints;
            var points = al.alloc(u32, 3);
            points[0] = plane_pts.items[0];
            points[1] = plane_pts.items[1];
            points[2] = plane_pts.items[2];
            try newfaces.append(al, .{ .normal = normal, .points = points });
        }

        if (plane_pts.items.len == 4) {
            const normal = v3.from_points(&[_]v3{
                data.points[plane_pts.items[0]],
                data.points[plane_pts.items[1]],
                data.points[plane_pts.items[2]],
            }) orelse return error.TriangulateInvalidPlanePoints;
            var points = al.alloc(u32, 3);
            points[0] = plane_pts.items[0];
            points[1] = plane_pts.items[1];
            points[2] = plane_pts.items[2];
            try newfaces.append(al, .{ .normal = normal, .points = points });
            points = al.alloc(u32, 3);
            points[0] = plane_pts.items[0];
            points[2] = plane_pts.items[2];
            points[3] = plane_pts.items[3];
            try newfaces.append(al, .{ .normal = normal, .points = points });
        }

        // Quads are trivial
        if (plane_pts.items.len == 4) {
            try newfaces.append(al, plane_pts.items[0]);
            try newfaces.append(al, plane_pts.items[1]);
            try newfaces.append(al, plane_pts.items[2]);
            try newfaces.append(al, plane_pts.items[0]);
            try newfaces.append(al, plane_pts.items[2]);
            try newfaces.append(al, plane_pts.items[3]);
            continue;
        }

        var start: u32 = 0;
        var end: u32 = @intCast(plane_pts.items.len - 1);
        var toggle = true;
        while (end - start >= 2) {
            var a: u32 = undefined;
            var b: u32 = undefined;
            var c: u32 = undefined;
            if (toggle) {
                a = start;
                b = start + 1;
                c = end;
                start += 1;
            } else {
                a = end;
                b = start;
                c = end - 1;
                end -= 1;
            }
            toggle = !toggle;
            try newfaces.append(al, plane_pts.items[a]);
            try newfaces.append(al, plane_pts.items[b]);
            try newfaces.append(al, plane_pts.items[c]);
        }
    }

    data.faces = newfaces.items;
}

// Returns list of n-gon faces
fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    var maybe_worldspawn: ?Map.Entity = null;
    worldspawn: for (map.entities) |ent| {
        for (ent.properties) |p| {
            if (streq(p.name, "classname") and streq(p.value, "worldspawn")) {
                maybe_worldspawn = ent;
                break :worldspawn;
            }
        }
    }

    // Do we really need to error? Maybe fallback to some other entity for
    // brush building?
    const worldspawn = maybe_worldspawn orelse {
        return error.InvalidMapNoWorldspawn;
    };

    // Assume most brushes will be rectanguloids of some sort
    const nplanes = worldspawn.brushes.len * 6;
    const npoints = nplanes * 4;
    var points = try std.ArrayListUnmanaged(v3).initCapacity(al, npoints);
    var planes = try std.ArrayListUnmanaged(Plane).initCapacity(al, nplanes);
    var faces = try std.ArrayListUnmanaged(GeoFace).initCapacity(al, nplanes);

    brush: for (worldspawn.brushes, 0..) |brush, brush_id| {
        // TODO: Maybe don't fail here
        const newplanes = try planes_from_brushplanes(al, brush.planes);

        if (newplanes.len < 3) {
            pr("Invalid brush (too few planes): brush #{}", .{brush_id});
            continue :brush;
        }

        for (0..newplanes.len) |plane_id| {
            const start = points.items.len;

            i: for (plane_id..newplanes.len) |i| {
                if (plane_id == i) continue :i;

                j: for (i..newplanes.len) |j| {
                    if (i == j) continue :j;

                    const a = newplanes[plane_id];
                    const b = newplanes[i];
                    const c = newplanes[j];
                    const pt = point_from_planes(a, b, c) orelse continue :j;

                    if (!point_in_planes(pt, newplanes)) continue :j;
                    try points.append(al, pt);
                }
            }

            const facepoints = points.items[start..];
            if (facepoints.len < 3) continue;

            var newface = GeoFace{
                .u = newplanes[plane_id].u,
                .v = newplanes[plane_id].v,
                .plane_id = @intCast(plane_id),
                .brush_id = @intCast(brush_id),
                .points = points.items,
                .point_ids = try al.alloc(u32, facepoints.len),
            };

            for (newface.point_ids, 0..) |*pt, pt_id| {
                pt.* = @intCast(start + pt_id);
            }

            newface.sort_points();
            try faces.append(al, newface);
        }

        try planes.appendSlice(al, newplanes);
    }

    return .{
        .planes = planes.items,
        .points = points.items,
        .faces = &[_]GeoFace{},
    };
}

fn will_split(_: Plane, _: []u32) bool {
    return false;
}

// Returns 0 for non-coplanar, 1 for same normal dir, -1 for opposite normal dir
fn coplanarity(_: Plane, _: GeoFace) i32 {
    return 0;
}

fn score_plane(plane_id: usize, data: GeoData) i32 {
    var num_splits: i32 = 0;
    const back: i32 = 0;
    const front: i32 = 0;

    for (data.faces) |face| {
        if (will_split(data.planes[plane_id], face)) {
            num_splits += 1;
            continue;
        }

        // if (coplanarity(plane, face) == 0) {
        //     // TODO: Check front/back
        // }
    }

    return (num_splits * 5) + (@abs(front - back));
}

fn build_bsp(data: *GeoData) Bsp {
    const candidate_plane = 0;
    const score = score_plane(0, data.*);
    _ = candidate_plane;
    _ = score;

    for (0..data.planes.len) |i| {
        const newscore = score_plane(data.planes[i], data);
        _ = newscore;
    }

    return .{};
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    _ = try discretize(al, map);

    // const bsp = build_bsp(&data);
    // dedupe_points(&data.points, data.faces);
    // try triangulate(al, &data);
    // points_to_obj("zbsp", data.points, data.faces);

    return .{};
    // return bsp;
}

test "points in plane" {
    // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
    const plane = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;

    const pts = &.{
        v3.make(2.0, 0.0, 0.0),
        v3.make(0.0, 2.0, 0.0),
        v3.make(0.0, 0.0, 2.0),
    };
    inline for (pts) |pt| {
        try std.testing.expect(point_in_planes(pt, &.{plane}));
    }
}

test "points outside plane" {
    // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
    const plane = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;

    const pts = &.{
        v3.make(0.0, 0.0, 0.0),
        v3.make(-4.0, 0.0, 0.0),
        v3.make(0.0, -4.0, 0.0),
        v3.make(0.0, 0.0, -4.0),
    };
    inline for (pts) |pt| {
        try std.testing.expect(!point_in_planes(pt, &.{plane}));
    }
}

test "point from planes" {
    const p1 = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;
    const p2 = Plane{ .normal = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .normal = v3.make(0.0, 0.0, 1.0), .offset = 0.0 };
    const result = point_from_planes(
        p1,
        p2,
        p3,
    ) orelse return error.InvalidPoint;

    const eps = std.math.floatEps(f32);
    try std.testing.expectApproxEqAbs(0.0, result.x, eps);
    try std.testing.expectApproxEqAbs(1.0, result.y, eps);
    try std.testing.expectApproxEqAbs(0.0, result.z, eps);
}

test "point from planes - throws error" {
    const p1 = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;
    const p2 = Plane{ .normal = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .normal = v3.make(1.0, 0.0, 0.0), .offset = 1.0 };

    try std.testing.expectEqual(null, point_from_planes(p1, p2, p3));
}

// test "dedupe points" {
//     var pt_data = [_]v3{
//         v3.make(0.0, 0.0, 0.0),
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(2.0, 0.0, 0.0),
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(3.0, 0.0, 0.0),
//         v3.make(5.0, 0.0, 0.0),
//     };
//     var pts: []v3 = pt_data[0..pt_data.len];

//     dedupe_points(&pts, &[_]GeoFace{});
//     try std.testing.expectEqualSlices(
//         v3,
//         &[_]v3{
//             v3.make(0.0, 0.0, 0.0),
//             v3.make(1.0, 0.0, 0.0),
//             v3.make(2.0, 0.0, 0.0),
//             v3.make(5.0, 0.0, 0.0),
//             v3.make(3.0, 0.0, 0.0),
//         },
//         pts,
//     );
// }

// test "dedupe points - face update" {
//     var pt_data = [_]v3{
//         v3.make(0.0, 0.0, 0.0),
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(2.0, 0.0, 0.0),
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(3.0, 0.0, 0.0),
//         v3.make(5.0, 0.0, 0.0),
//     };
//     var pts: []v3 = pt_data[0..pt_data.len];
//     var facepoints = [_]u32{ 0, 1, 2, 3, 4, 5 };
//     var facelist = [_]GeoFace{GeoFace{
//         .plane_id = 0,
//         .brush_id = 0,
//         .u = return error.NotYet,
//         .v = return error.NotYet,
//         .points = &facepoints,
//     }};
//     const faces: []GeoFace = &facelist;

//     dedupe_points(&pts, faces);
//     try std.testing.expectEqualSlices(
//         v3,
//         &[_]v3{
//             v3.make(0.0, 0.0, 0.0),
//             v3.make(1.0, 0.0, 0.0),
//             v3.make(2.0, 0.0, 0.0),
//             v3.make(5.0, 0.0, 0.0),
//             v3.make(3.0, 0.0, 0.0),
//         },
//         pts,
//     );
//     try std.testing.expectEqualSlices(
//         u32,
//         &[_]u32{ 0, 1, 2, 1, 4, 3 },
//         faces[0].points,
//     );
// }
