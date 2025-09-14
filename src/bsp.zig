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
    normal_id: u32 = 0,
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
    faces: []u32,
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
        plane.norm.dot(pt) + plane.offset,
        0.0,
        ON_EPSILON,
    );
}

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    for (planes, 0..) |plane, i| {
        _ = i;
        const pt_offset = plane.norm.dot(pt) + plane.offset;

        const on_plane = std.math.approxEqAbs(f32, pt_offset, 0.0, ON_EPSILON);
        // The plane halfspaces that construct a solid shape define the solid
        // portion as being 'in front of the plane' rather than behind
        const within_volume = pt_offset > 0.0;

        result = result and (within_volume or on_plane);
    }

    return result;
}

fn point_from_planes(a: Plane, b: Plane, c: Plane) ?v3 {
    const c1 = b.norm.cross(c.norm);
    const c2 = c.norm.cross(a.norm);
    const c3 = a.norm.cross(b.norm);

    const numerator =
        c1.mul(-a.offset).add(c2.mul(-b.offset)).add(c3.mul(-c.offset));
    const denominator = a.norm.dot(c1);

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

const PointComparator = struct {
    const Context = struct {
        u: v3,
        v: v3,
        centroid: v3,
        points: []v3,
    };

    fn less_than(ctx: Context, lhs: u32, rhs: u32) bool {
        const u = ctx.u;
        const v = ctx.v;
        const pts = ctx.points;

        const lhsx = u.dot(pts[lhs]);
        const lhsy = v.dot(pts[lhs]);
        const rhsx = u.dot(pts[rhs]);
        const rhsy = v.dot(pts[rhs]);
        const cx = u.dot(ctx.centroid);
        const cy = v.dot(ctx.centroid);

        const lhs_angle = std.math.atan2(lhsy - cy, lhsx - cx);
        const rhs_angle = std.math.atan2(rhsy - cy, rhsx - cx);

        return lhs_angle < rhs_angle;
    }
};

fn dedupe_points(data: *GeoData) void {
    if (data.points.len == 0) return;

    // Index of the last element
    var end = data.points.len - 1;

    for (0..end) |i| {
        var j = i + 1;
        while (j < end + 1) : (j += 1) {
            if (!data.points[i].eq(data.points[j], VERTEX_EPSILON)) continue;

            // TODO: Probably put all the point replacements in a HashMap and
            // iterate over faces a single time
            for (data.faces) |*pt_index| {
                if (pt_index.* == @as(u32, @intCast(j))) {
                    pt_index.* = @as(u32, @intCast(i));
                }
            }

            if (j == end) {
                end -= 1;
                continue;
            }

            const tmp = data.points[j];
            data.points[j] = data.points[end];
            data.points[end] = tmp;

            end -= 1;
            j -= 1;
        }
    }

    data.points = data.points[0 .. end + 1];
}

fn triangulate(al: std.mem.Allocator, data: *GeoData) !void {
    var newfaces = try std.ArrayListUnmanaged(u32).initCapacity(
        al,
        data.faces.len,
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

        if (plane_pts.items.len < 3) continue;
        if (plane_pts.items.len == 3) {
            try newfaces.append(al, plane_pts.items[0]);
            try newfaces.append(al, plane_pts.items[1]);
            try newfaces.append(al, plane_pts.items[2]);
            continue;
        }

        var centroid = data.points[plane_pts.items[0]];
        for (plane_pts.items[1..]) |pt| centroid = centroid.add(
            data.points[pt],
        );
        centroid = centroid.div(@floatFromInt(plane_pts.items.len));

        std.sort.pdq(
            u32,
            plane_pts.items,
            PointComparator.Context{
                .u = data.planes[plane_id].u,
                .v = data.planes[plane_id].v,
                .centroid = centroid,
                .points = data.points,
            },
            PointComparator.less_than,
        );

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
    const faces = try std.ArrayListUnmanaged(u32).initCapacity(al, 1024);

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

    brush: for (worldspawn.brushes, 0..) |brush, brush_i| {
        const newplanes = try planes_from_brushplanes(al, brush.planes);

        if (newplanes.len < 3) {
            pr("Invalid brush (too few planes): brush #{}", .{brush_i});
            continue :brush;
        }

        for (0..newplanes.len) |i| {
            j: for (i..newplanes.len) |j| {
                if (i == j) continue :j;

                k: for (j..newplanes.len) |k| {
                    if (j == k) continue :k;

                    const a = newplanes[i];
                    const b = newplanes[j];
                    const c = newplanes[k];
                    const pt = point_from_planes(a, b, c) orelse continue;

                    if (!point_in_planes(pt, newplanes)) continue;
                    try points.append(al, pt);
                }
            }
        }

        try planes.appendSlice(al, newplanes);
    }

    return .{
        .planes = planes.items,
        .points = points.items,
        .faces = faces.items,
    };
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    var data = try discretize(al, map);

    dedupe_points(&data);
    try triangulate(al, &data);
    points_to_obj("zbsp", data.points, data.faces);

    return .{};
}

test "points in plane" {
    // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
    const plane = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;

    const pts = &.{
        v3.make(0.0, 0.0, 0.0),
        v3.make(-2.0, 0.0, 0.0),
        v3.make(0.0, -2.0, 0.0),
        v3.make(0.0, 0.0, -2.0),
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
        v3.make(2.0, 0.0, 0.0),
        v3.make(0.0, 2.0, 0.0),
        v3.make(0.0, 0.0, 2.0),
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
    const p2 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .norm = v3.make(0.0, 0.0, 1.0), .offset = 0.0 };
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
    const p2 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 1.0 };

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

//     pts = dedupe_points(pts);
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
