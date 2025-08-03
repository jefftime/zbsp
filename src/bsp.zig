const std = @import("std");
const Map = @import("map.zig").Map;
const v3 = @import("geo.zig").v3;
const Plane = @import("geo.zig").Plane;
const Mat3 = @import("math.zig").Mat3;

const pr = std.debug.print;

pub const Bsp = struct {
    normal_id: u32 = 0,
    front: ?*Bsp = null,
    back: ?*Bsp = null,

    pub fn print_digraph(self: *const Bsp) void {
        _ = self;
        std.log.debug("Todo!", .{});
    }
};

const Poly = struct {
    face_id: u32,
};

const GeoData = struct {
    points: []v3,
    normals: []v3,
    faces: [][3]u32,
};

fn planes_from_brushplanes(al: std.mem.Allocator, bps: []Map.Plane) ![]Plane {
    var planes = try std.ArrayListUnmanaged(Plane).initCapacity(al, bps.len);
    for (bps) |bp| {
        try planes.append(al, Plane.from_points(
            bp.a,
            bp.b,
            bp.c,
        ) orelse return error.InvalidPlane);
    }

    return planes.items;
}

fn point_on_plane(pt: v3, plane: Plane) bool {
    const eps = std.math.floatEps(f32);
    return std.math.approxEqAbs(
        f32,
        plane.norm.dot(pt) + plane.offset,
        0.0,
        eps,
    );
}

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    for (planes, 0..) |plane, i| {
        const eps = std.math.floatEps(f32);
        const pt_offset = plane.norm.dot(pt) + plane.offset;

        const on_plane = std.math.approxEqAbs(f32, pt_offset, 0.0, eps);
        // The plane halfspaces that construct a solid shape define the solid
        // portion as being 'in front of the plane' rather than behind
        const within_volume = pt_offset > 0.0;

        result = result and (within_volume or on_plane);
        if (!result) {
            pr("Point {{ {d:.1} {d:.1} {d:.1} }} failed with plane {}\n", .{
                pt.x,
                pt.y,
                pt.z,
                i,
            });
        }
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

    if (std.math.approxEqAbs(f32, denominator, 0.0, std.math.floatEps(f32))) {
        return null;
    }

    return numerator.div(denominator);
}

fn points_to_obj(points: []v3, faces: []u32) void {
    pr("o Points\n", .{});
    for (points) |pt| pr("v {d:.1} {d:.1} {d:.1}\n", .{ pt.x, pt.y, pt.z });
    var i: u32 = 0;
    while (i < faces.len) : (i += 3) {
        pr("f {} {} {}\n", .{ faces[i], faces[i + 1], faces[i + 2] });
    }
}

const PointComparator = struct {
    const Context = struct {
        u: v3,
        v: v3,
        centroid: v3,
        points: []v3,
    };

    fn lessThan(ctx: Context, lhs: u32, rhs: u32) bool {
        const u = ctx.u;
        const v = ctx.v;
        const c = ctx.centroid;
        const pts = ctx.points;

        // NOTE: I'm lazy and don't want to make a Vec2 yet
        const lhs2d = v3.make(u.dot(pts[lhs]), v.dot(pts[lhs]), 0.0);
        const rhs2d = v3.make(u.dot(pts[rhs]), v.dot(pts[rhs]), 0.0);

        const lhs_angle = std.math.atan2(lhs2d.y - c.y, lhs2d.x - c.x);
        const rhs_angle = std.math.atan2(rhs2d.y - c.y, rhs2d.x - c.x);

        return lhs_angle < rhs_angle;
    }
};

fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    var points = try std.ArrayListUnmanaged(v3).initCapacity(al, 1024);
    var faces = try std.ArrayListUnmanaged(u32).initCapacity(al, 1024);

    // Collect points
    // TODO: We're probably going to want to only get the worldspawn brushes at
    // some point instead of getting all the brushes across all entities
    for (map.entities) |ent| {
        brush: for (ent.brushes, 0..) |brush, brush_i| {
            const planes = try planes_from_brushplanes(al, brush.planes);

            if (planes.len < 3) {
                pr("Invalid brush (too few planes): brush #{}", .{brush_i});
                continue :brush;
            }

            for (0..planes.len) |i| {
                j: for (i..planes.len) |j| {
                    if (i == j) continue :j;

                    k: for (j..planes.len) |k| {
                        if (j == k) continue :k;

                        const a = planes[i];
                        const b = planes[j];
                        const c = planes[k];
                        const pt = point_from_planes(a, b, c) orelse continue;

                        // TODO: Maybe we just need to check if point is in
                        // [a, b, c] only?
                        if (!point_in_planes(pt, planes)) continue;
                        try points.append(al, pt);
                    }
                }
            }

            // List of u32 indices into the points array
            var plane_pts = try std.ArrayListUnmanaged(u32).initCapacity(
                al,
                points.items.len,
            );
            // Trenchbroom defines up as +Z
            const up = v3.make(0.0, 0.0, 1.0);
            for (planes) |plane| {
                // TODO: If this is a bottleneck we should look at using a
                // dictionary to store plane-point relationships somehow
                plane_pts.clearRetainingCapacity();
                for (points.items, 0..) |pt, i| {
                    if (!point_on_plane(pt, plane)) continue;
                    try plane_pts.append(al, @intCast(i));
                }

                // Trivial cases
                if (plane_pts.items.len == 3) {
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[1]);
                    try faces.append(al, plane_pts.items[2]);
                    continue;
                }
                if (plane_pts.items.len == 4) {
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[1]);
                    try faces.append(al, plane_pts.items[2]);
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[2]);
                    try faces.append(al, plane_pts.items[3]);
                    continue;
                }

                // Complex cases
                var centroid = v3.make(0.0, 0.0, 0.0);
                for (plane_pts.items) |pt| centroid = centroid.add(
                    points.items[pt],
                );
                centroid = centroid.div(@floatFromInt(plane_pts.items.len));

                const u = plane.norm.cross(up).norm();
                const v = plane.norm.cross(u).norm();

                std.sort.pdq(
                    u32,
                    plane_pts.items,
                    PointComparator.Context{
                        .u = u,
                        .v = v,
                        .centroid = centroid,
                        .points = points.items,
                    },
                    PointComparator.lessThan,
                );

                var left: u32 = 0;
                var right: u32 = @intCast(plane_pts.items.len - 1);
                var toggle = true;
                while (right - left >= 2) {
                    var a: u32 = undefined;
                    var b: u32 = undefined;
                    var c: u32 = undefined;
                    if (toggle) {
                        a = left;
                        b = left + 1;
                        c = right;
                        left += 1;
                    } else {
                        a = right;
                        b = left;
                        c = right - 1;
                        right -= 1;
                    }
                    toggle = !toggle;
                    try faces.append(al, plane_pts.items[a]);
                    try faces.append(al, plane_pts.items[b]);
                    try faces.append(al, plane_pts.items[c]);
                }
            }
        }
    }

    // Construct faces
    points_to_obj(points.items, faces.items);

    return .{
        .points = &.{},
        .faces = &.{},
        .normals = &.{},
    };
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    const data = try discretize(al, map);
    _ = data;

    return .{};
}

// test "points in plane" {
//     // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
//     const plane = Plane.from_points(
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(0.0, 1.0, 0.0),
//         v3.make(0.0, 0.0, 1.0),
//     ) orelse return error.InvalidPlane;

//     const pts = &.{
//         v3.make(0.0, 0.0, 0.0),
//         v3.make(-2.0, 0.0, 0.0),
//         v3.make(0.0, -2.0, 0.0),
//         v3.make(0.0, 0.0, -2.0),
//     };
//     inline for (pts) |pt| {
//         try std.testing.expect(point_in_planes(pt, &.{plane}));
//     }
// }

// test "points outside plane" {
//     // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
//     const plane = Plane.from_points(
//         v3.make(1.0, 0.0, 0.0),
//         v3.make(0.0, 1.0, 0.0),
//         v3.make(0.0, 0.0, 1.0),
//     ) orelse return error.InvalidPlane;

//     const pts = &.{
//         v3.make(2.0, 0.0, 0.0),
//         v3.make(0.0, 2.0, 0.0),
//         v3.make(0.0, 0.0, 2.0),
//     };
//     inline for (pts) |pt| {
//         try std.testing.expect(!point_in_planes(pt, &.{plane}));
//     }
// }

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
