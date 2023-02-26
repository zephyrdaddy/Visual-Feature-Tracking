load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_library(
	name = "v_frontend_lib",
	srcs = glob(["*.cpp"], exclude = ["VisualFrontend.cpp"]),
    hdrs = glob(["*.hpp"]),
    copts = [""],
    deps = ["@opencv//:opencv", "@boost//:boost", "@eigen//:eigen",],
    visibility = ["//visibility:public"],
	linkstatic = 1,
	)

cc_binary(
	name = "v_frontend",
	srcs = ["VisualFrontend.cpp"],
    # includes = ["*.hpp"],
    deps = ["@opencv//:opencv", 
            "@boost//:boost", 
            "@rapid_json//:rapid_json", 
            "//src:v_frontend_lib"],
	)

    