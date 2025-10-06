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
    name: []const u8 = "Map",
    planes: []Plane,
    points: []v3,
    faces: []Face,

    pub fn to_obj(self: *const GeoData) void {
        pr("o {s}\n", .{self.name});
        for (self.points) |pt| pr("v {d:.1} {d:.1} {d:.1}\n", .{
            pt.x, pt.y, pt.z,
        });
        for (self.faces) |face| {
            pr("f ", .{});
            for (face.point_ids) |pt| pr("{} ", .{pt + 1});
            pr("\n", .{});
        }
    }
};

const Face = struct {
    u: v3,
    v: v3,
    plane_id: u32,
    brush_id: u32,
    point_ids: []u32,
    // points: []v3,

    pub fn sort_points(self: *Face, points: []v3) void {
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

        var centroid = points[self.point_ids[0]];
        for (self.point_ids[1..]) |pt_id| {
            centroid = centroid.add(points[pt_id]);
        }
        centroid = centroid.div(@floatFromInt(self.point_ids.len));

        std.sort.pdq(
            u32,
            self.point_ids,
            PointComparator.Context{
                .u = self.u,
                .v = self.v,
                .centroid = centroid,
                .points = points,
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

    return try planes.toOwnedSlice(al);
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

fn dedupe_points(points: *[]v3, faces: *[]Face) void {
    if (points.len == 0) return;

    // Index of the last element
    var end = points.len - 1;

    for (0..end) |i| {
        var j = i + 1;
        while (j < end + 1) : (j += 1) {
            if (!points.*[i].eq(points.*[j], VERTEX_EPSILON)) continue;

            // TODO: Probably put all the point replacements in a HashMap and
            // iterate over faces a single time
            for (faces.*) |*face| {
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
    var newfaces = std.ArrayListUnmanaged(Face).fromOwnedSlice(data.faces);
    try newfaces.ensureTotalCapacity(al, newfaces.items.len * 2);

    const newfacelen = newfaces.items.len;
    face: for (0..newfacelen) |i| {
        if (newfaces.items[i].point_ids.len == 3) continue :face;
        if (newfaces.items[i].point_ids.len == 4) {
            var newpoint = try al.alloc(u32, 3);
            newpoint[0] = newfaces.items[i].point_ids[0];
            newpoint[1] = newfaces.items[i].point_ids[2];
            newpoint[2] = newfaces.items[i].point_ids[3];
            newfaces.items[i].point_ids = newfaces.items[i].point_ids[0..3];
            continue :face;
        }

        const ntris = newfaces.items[i].point_ids.len - 2;
        var newpoints = try al.alloc(u32, ntris * 3);

        // This will create a triangle span instead of fan. It assembles it by
        // switching between making a triangle from the front, then the back,
        // etc and removing a point from the list each time
        var start: u32 = 0;
        var end: u32 = @intCast(newfaces.items[i].point_ids.len - 1);
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

            newpoints[0] = newfaces.items[i].point_ids[a];
            newpoints[1] = newfaces.items[i].point_ids[b];
            newpoints[2] = newfaces.items[i].point_ids[c];

            try newfaces.append(al, .{
                .brush_id = newfaces.items[i].brush_id,
                .plane_id = newfaces.items[i].plane_id,
                .u = newfaces.items[i].u,
                .v = newfaces.items[i].v,
                .point_ids = newpoints[0..3],
            });

            newpoints = newpoints[3..];
        }
    }
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
    const planecount = worldspawn.brushes.len * 6;
    const pointcount = (planecount * 4) * 12 / 10; // 1.2x for a little leeway
    var points = try std.ArrayListUnmanaged(v3).initCapacity(
        al,
        pointcount,
    );
    var planes = try std.ArrayListUnmanaged(Plane).initCapacity(al, planecount);
    var faces = try std.ArrayListUnmanaged(Face).initCapacity(al, planecount);

    brush: for (worldspawn.brushes, 0..) |brush, brush_id| {
        // Key is an index into the points array, value is the three faces from
        // the plane intersection formula
        //
        // K - point_id
        // V - { newplane_id, newplane_id, newplane_id }
        var plane_points: std.AutoHashMapUnmanaged(u32, [3]u32) = .empty;
        defer plane_points.deinit(al);

        const newplanes = planes_from_brushplanes(al, brush.planes) catch {
            continue :brush;
        };

        if (newplanes.len < 3) {
            pr("Invalid brush (too few planes): brush #{}", .{brush_id});
            continue :brush;
        }

        // Build up the points
        for (0..newplanes.len) |newplane_id| {
            i: for (newplane_id..newplanes.len) |i| {
                if (newplane_id == i) continue :i;

                j: for (i..newplanes.len) |j| {
                    if (i == j) continue :j;

                    const a = newplanes[newplane_id];
                    const b = newplanes[i];
                    const c = newplanes[j];
                    const pt = point_from_planes(a, b, c) orelse continue :j;

                    if (!point_in_planes(pt, newplanes)) continue :j;
                    const curpoint: u32 = @intCast(points.items.len);
                    try points.append(al, pt);
                    try plane_points.put(
                        al,
                        curpoint,
                        .{ @intCast(newplane_id), @intCast(i), @intCast(j) },
                    );
                }
            }
        }

        for (newplanes, 0..) |newplane, newplane_id| {
            var facepoints = try std.ArrayListUnmanaged(u32).initCapacity(
                al,
                plane_points.size,
            );
            var pt_iter = plane_points.iterator();
            pt: while (pt_iter.next()) |pt| {
                const is_first = newplane_id == pt.value_ptr.*[0];
                const is_second = newplane_id == pt.value_ptr.*[1];
                const is_third = newplane_id == pt.value_ptr.*[2];
                if (!(is_first or is_second or is_third)) continue :pt;

                try facepoints.append(al, pt.key_ptr.*);
            }
            if (facepoints.items.len == 0) continue;

            var newface = Face{
                .brush_id = @intCast(brush_id),
                .plane_id = @intCast(planes.items.len + newplane_id),
                .u = newplane.u,
                .v = newplane.v,
                .point_ids = try facepoints.toOwnedSlice(al),
            };
            newface.sort_points(points.items);
            try faces.append(al, newface);
        }

        try planes.appendSlice(al, newplanes);
    }

    return .{
        .planes = try planes.toOwnedSlice(al),
        .points = try points.toOwnedSlice(al),
        .faces = try faces.toOwnedSlice(al),
    };
}

fn will_split(_: Plane, _: []u32) bool {
    return false;
}

// Returns 0 for non-coplanar, 1 for same normal dir, -1 for opposite normal dir
fn coplanarity(_: Plane, _: Face) i32 {
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
    var data = try discretize(al, map);

    // _ = build_bsp(&data);
    // dedupe_points(&data.points, &data.faces);
    // try triangulate(al, &data);
    data.to_obj();

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

test "dedupe points" {
    var pt_data = [_]v3{
        v3.make(0.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(2.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(3.0, 0.0, 0.0),
        v3.make(5.0, 0.0, 0.0),
    };
    var pts: []v3 = pt_data[0..pt_data.len];

    var empty_faces: []Face = &[_]Face{};
    dedupe_points(&pts, &empty_faces);
    try std.testing.expectEqualSlices(
        v3,
        &[_]v3{
            v3.make(0.0, 0.0, 0.0),
            v3.make(1.0, 0.0, 0.0),
            v3.make(2.0, 0.0, 0.0),
            v3.make(5.0, 0.0, 0.0),
            v3.make(3.0, 0.0, 0.0),
        },
        pts,
    );
}

test "dedupe points - face update" {
    var pt_data = [_]v3{
        v3.make(0.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(2.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(3.0, 0.0, 0.0),
        v3.make(5.0, 0.0, 0.0),
    };
    var pts: []v3 = pt_data[0..];
    var facepoints = [_]u32{ 0, 1, 2, 3, 4, 5 };
    var facelist = [_]Face{Face{
        .plane_id = 0,
        .brush_id = 0,
        .u = undefined,
        .v = undefined,
        .point_ids = facepoints[0..],
    }};
    var faces: []Face = facelist[0..];

    dedupe_points(&pts, &faces);
    try std.testing.expectEqualSlices(
        v3,
        &[_]v3{
            v3.make(0.0, 0.0, 0.0),
            v3.make(1.0, 0.0, 0.0),
            v3.make(2.0, 0.0, 0.0),
            v3.make(5.0, 0.0, 0.0),
            v3.make(3.0, 0.0, 0.0),
        },
        pts,
    );
    try std.testing.expectEqualSlices(
        u32,
        &[_]u32{ 0, 1, 2, 1, 4, 3 },
        faces[0].point_ids,
    );
}
