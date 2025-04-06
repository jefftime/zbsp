const scanner = @import("scanner.zig");
const util = @import("util.zig");
const std = @import("std");
const limits = @import("limits.zig");

comptime {
    const builtin = @import("builtin");
    if (!builtin.is_test) @export(&main, .{ .name = "main" });
}

pub fn main(_: c_int, _: [*c][*c]c_char) callconv(.c) c_int {
    var al = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer al.deinit();

    var file = std.fs.cwd().openFile(
        "mapfile.map",
        .{ .mode = .read_only },
    ) catch |err| util.die(err, "Failed to open mapfile.map", .{});
    // const buf = file.readToEndAlloc(
    //     al.allocator(),
    //     limits.MAX_FILE_SIZE,
    // ) catch |err| util.die(err, "Failed to read mapfile.map", .{});

    const tokens = scanner.scan_buf(
        al.allocator(),
        "-.123 .123 123 -123 12.3",
        .{},
    ) catch |err| util.die(err, "Failed to scan file", .{});
    file.close();
    for (tokens.items) |tk| tk.print();

    return 0;
}

test {
    std.testing.refAllDecls(@This());
}
