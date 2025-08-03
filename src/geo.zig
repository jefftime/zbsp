const std = @import("std");

pub const Plane = struct {
    norm: v3,
    // We use the general/graphics form of nx + d = 0
    offset: f32,

    pub fn from_points(a: v3, b: v3, c: v3) ?Plane {
        const ab = b.sub(a);
        const ac = c.sub(a);

        const norm = ab.cross(ac).norm();

        return .{
            .norm = norm,
            .offset = -norm.dot(a),
        };
    }
};

test "plane from points" {
    const plane = Plane.from_points(
        v3.make(1, 0, 0),
        v3.make(0, 1, 0),
        v3.make(0, 0, 1),
    ) orelse return error.InvalidPlane;
    const eps = std.math.floatEps(f32);

    try std.testing.expectApproxEqAbs(plane.norm.x, 1.0 / @sqrt(3.0), eps);
    try std.testing.expectApproxEqAbs(plane.norm.y, 1.0 / @sqrt(3.0), eps);
    try std.testing.expectApproxEqAbs(plane.norm.z, 1.0 / @sqrt(3.0), eps);
    try std.testing.expectApproxEqAbs(plane.offset, -1.0 / @sqrt(3.0), eps);
}

pub const v3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn make(x: f32, y: f32, z: f32) v3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn norm(self: v3) v3 {
        const m = self.mag();

        return if (std.math.approxEqAbs(f32, m, 0.0, std.math.floatEps(f32)))
            .{ .x = 0, .y = 0, .z = 0 }
        else
            .{ .x = self.x / m, .y = self.y / m, .z = self.z / m };
    }

    pub fn mag(self: v3) f32 {
        return @sqrt(self.x * self.x + self.y * self.y + self.z * self.z);
    }

    pub fn cross(self: v3, other: v3) v3 {
        const i = self.y * other.z - self.z * other.y;
        const j = self.z * other.x - self.x * other.z;
        const k = self.x * other.y - self.y * other.x;

        return .{ .x = i, .y = j, .z = k };
    }

    pub fn dot(self: v3, other: v3) f32 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }

    pub fn add(self: v3, other: v3) v3 {
        return .{
            .x = self.x + other.x,
            .y = self.y + other.y,
            .z = self.z + other.z,
        };
    }

    pub fn sub(self: v3, other: v3) v3 {
        return .{
            .x = self.x - other.x,
            .y = self.y - other.y,
            .z = self.z - other.z,
        };
    }

    pub fn mul(self: v3, rhs: f32) v3 {
        return .{ .x = self.x * rhs, .y = self.y * rhs, .z = self.z * rhs };
    }

    pub fn div(self: v3, denom: f32) v3 {
        return .{
            .x = self.x / denom,
            .y = self.y / denom,
            .z = self.z / denom,
        };
    }
};

test "v3 mul" {
    const v = v3.make(1.0, 2.0, 3.0);
    const result = v.mul(2);
    const eps = std.math.floatEps(f32);

    try std.testing.expectApproxEqAbs(2.0, result.x, eps);
    try std.testing.expectApproxEqAbs(4.0, result.y, eps);
    try std.testing.expectApproxEqAbs(6.0, result.z, eps);
}

test "v3 div" {
    const v = v3.make(1.0, 2.0, 3.0);
    const result = v.div(2.0);
    const eps = std.math.floatEps(f32);

    try std.testing.expectApproxEqAbs(0.5, result.x, eps);
    try std.testing.expectApproxEqAbs(1.0, result.y, eps);
    try std.testing.expectApproxEqAbs(1.5, result.z, eps);
}

test "v3 magnitude (2D)" {
    const v = v3.make(3, 4, 0);
    try std.testing.expectEqual(5, v.mag());
    try std.testing.expectApproxEqAbs(5.0, v.mag(), std.math.floatEps(f32));
}

test "v3 magnitude (3D)" {
    const v = v3.make(3, 4, 5);
    try std.testing.expectApproxEqAbs(
        7.0710678118654,
        v.mag(),
        std.math.floatEps(f32),
    );
}

test "v3 cross" {
    const v1 = v3.make(1, 2, 3);
    const v2 = v3.make(4, 5, 6);
    const result = v1.cross(v2);
    try std.testing.expectApproxEqAbs(-3.0, result.x, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(6.0, result.y, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(-3.0, result.z, std.math.floatEps(f32));
}

test "v3 add" {
    const v1 = v3.make(1, 2, 3);
    const v2 = v3.make(4, 5, 6);
    const result = v1.add(v2);
    try std.testing.expectApproxEqAbs(5.0, result.x, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(7.0, result.y, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(9.0, result.z, std.math.floatEps(f32));
}

test "v3 sub" {
    const v1 = v3.make(1, 2, 3);
    const v2 = v3.make(4, 5, 6);
    const result = v1.sub(v2);
    try std.testing.expectApproxEqAbs(-3.0, result.x, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(-3.0, result.y, std.math.floatEps(f32));
    try std.testing.expectApproxEqAbs(-3.0, result.z, std.math.floatEps(f32));
}

test "v3 cross (coplanar points)" {
    const a = v3.make(0.0, 0.0, 0.0);
    const b = v3.make(1.0, 0.0, 0.0);
    const c = v3.make(0.0, 0.0, 1.0);
    const ab = b.sub(a);
    const ac = c.sub(a);
    const result = ab.cross(ac).norm();
    const eps = std.math.floatEps(f32);
    try std.testing.expectApproxEqAbs(0.0, result.x, eps);
    try std.testing.expectApproxEqAbs(-1.0, result.y, eps);
    try std.testing.expectApproxEqAbs(0.0, result.z, eps);
}

test "v3 dot" {
    const a = v3.make(0, 1, 2);
    const b = v3.make(3, 4, 5);
    const result = a.dot(b);

    const eps = std.math.floatEps(f32);
    try std.testing.expectApproxEqAbs(14, result, eps);
}

test "v3 dot 2" {
    const a = v3.make(0, -1, 2);
    const b = v3.make(3, 4, 5);
    const result = a.dot(b);

    const eps = std.math.floatEps(f32);
    try std.testing.expectApproxEqAbs(6, result, eps);
}
