#pragma once
namespace forlin {
	enum class POINTTYPE {
		XYZ,
		XYZRGB,
		XYZRGBI,
		XYZRGBNXNYNZ
	};

	enum class FUNCTION {
		ConvertToFYData,
		DownSample
	};
}