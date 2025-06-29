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

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    pr("incoming pt: {d} {d} {d}\n", .{ pt.x, pt.y, pt.z });
    for (planes, 0..) |plane, i| {
        const eps = std.math.floatEps(f32);
        const pt_offset = plane.norm.dot(pt) + plane.offset;
        const inside =
            pt_offset < 0 or std.math.approxEqAbs(f32, pt_offset, 0.0, eps);
        if (!inside) {
            pr("plane: {}\n", .{plane});
            pr("pt: {}\n", .{pt});
            // pr("pt_offset: {d}\n", .{pt_offset});
            // pr(
            //     "plane.norm: {d} {d} {d}\n",
            //     .{ plane.norm.x, plane.norm.y, plane.norm.z },
            // );
            // pr("plane.offset: {d}\n", .{plane.offset});
            // pr("plane.norm.dot(pt): {d}\n", .{plane.norm.dot(pt)});
            return false;
        } else {
            pr("plane #{}: good\n", .{i});
        }

        result = result and inside;
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

fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    // const points = try std.ArrayListUnmanaged(f32).initCapacity(al, 1024);
    // const eps = std.math.floatEps(f32);
    const norms: std.AutoHashMapUnmanaged(usize, v3) = .empty;
    _ = norms;

    for (map.entities) |ent| {
        brush: for (ent.brushes, 0..) |brush, brush_i| {
            pr("Brush {}:\n", .{brush_i});

            const planes = try planes_from_brushplanes(al, brush.planes);
            if (planes.len < 3) {
                pr("Invalid brush (too few planes): {}", .{brush_i});
                break :brush;
            }

            pr(" Points:\n", .{});
            for (0..planes.len - 2) |i| {
                for (i + 1..planes.len) |j| {
                    for (j + 1..planes.len) |k| {
                        pr("{} {} {}\n", .{ i, j, k });

                        const a = planes[i];
                        const b = planes[j];
                        const c = planes[k];
                        const pt = point_from_planes(a, b, c) orelse {
                            pr("Invalid point\n", .{});
                            continue;
                        };

                        _ = point_in_planes(pt, planes);
                    }
                }
            }

            // pr("Brush: {}\n", .{brush_i});
            // for (planes) |pl| pr("  {}\n", .{pl});
        }
    }

    // for (map.entities) |ent| {
    //     for (ent.brushes) |brush| {
    //         const planes = try planes_from_brushplanes(al, brush.planes);
    //         for (planes) |a| {
    //             for (planes[1..]) |b| {
    //                 for (planes[2..]) |c| {
    //                     pr("{} {} {}", .{ a, b, c });
    //                 }
    //             }
    //         }
    //     }
    // }

    // outer: for (map.entities) |ent| {
    //     for (ent.brushes) |brush| {
    //         for (brush.planes, 0..) |plane, i| {
    //             const planenorm = norms.get(i) orelse blk: {
    //                 const norm = plane_norm(plane);
    //                 try norms.put(al, i, norm);
    //                 break :blk norm;
    //             };

    //             pr("{} planenorm: {}\n", .{ i, planenorm });

    //             for (i + 1..brush.planes.len) |j| {
    //                 const other_planenorm = norms.get(j) orelse blk: {
    //                     const norm = plane_norm(brush.planes[j]);
    //                     try norms.put(al, j, norm);
    //                     break :blk norm;
    //                 };

    //                 const normcross = planenorm.cross(other_planenorm);
    //                 pr("{} normcross {}: {}\n", .{ i, j, normcross });
    //                 // Skip parallel planes
    //                 if (normcross.mag() < eps) {
    //                     pr("WARN: {}: skipping index {} (parallel)\n", .{
    //                         i,
    //                         j,
    //                     });
    //                     continue;
    //                 }
    //             }
    //         }
    //         break :outer;
    //     }
    // }

    return error.NotYet;

    // return .{
    //     .points = &.{},
    //     .faces = &.{},
    //     .normals = &.{},
    // };
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    const data = try discretize(al, map);
    _ = data;

    return error.NotYet;
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
