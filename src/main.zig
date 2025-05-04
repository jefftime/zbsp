const scanner = @import("scanner.zig");
const util = @import("util.zig");
const std = @import("std");
const limits = @import("limits.zig");
const map = @import("map.zig");
const bsp = @import("bsp.zig");

comptime {
    const builtin = @import("builtin");
    if (!builtin.is_test) @export(&main, .{ .name = "main" });
}

pub fn main(_: c_int, _: [*c][*c]c_char) callconv(.c) c_int {
    var arena_allocator = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena_allocator.deinit();
    const al = arena_allocator.allocator();

    var file = std.fs.cwd().openFile(
        "mapfile.map",
        .{ .mode = .read_only },
    ) catch |err| util.die(err, "Failed to open mapfile.map", .{});
    const buf = file.readToEndAlloc(
        al,
        limits.MAX_FILE_SIZE,
    ) catch |err| util.die(err, "Failed to read mapfile.map", .{});

    const tokens = scanner.scan_buf(
        al,
        buf,
        .{ .double_slash_comments = true },
    ) catch |err| util.die(err, "Failed to scan file", .{});
    // for (tokens.items) |tk| tk.print();
    file.close();

    std.log.debug("fuck 3", .{});
    const m = map.parse(al, tokens.items) catch |err| util.die(
        err,
        "{}: Failed to parse map",
        .{err},
    );
    m.print();

    // const b = bsp.compile(al, m) catch |err| util.die(
    //     err,
    //     "Failed to compile BSP",
    //     .{},
    // );
    // b.print_digraph();

    _ = arena_allocator.reset(.free_all);

    return 0;
}

test {
    std.testing.refAllDecls(@This());
}
