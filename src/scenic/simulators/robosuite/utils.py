"""Conversions between Scenic and robosuite coordinates."""


def scenicToRobosuitePosition(pos):
    raise NotImplementedError
    # Reference: scenicToMetaDrivePosition (metadrive/utils.py); robosuite is
    # 3D, so z would carry through as well:
    #
    # def scenicToMetaDrivePosition(vec, scenic_offset):
    #     adjusted_x = vec[0] - scenic_offset[0]
    #     adjusted_y = vec[1] - scenic_offset[1]
    #     return adjusted_x, adjusted_y


def robosuiteToScenicPosition(pos):
    raise NotImplementedError
    # Reference: metadriveToScenicPosition (metadrive/utils.py):
    #
    # def metadriveToScenicPosition(loc, scenic_offset):
    #     x_scenic = loc[0] + scenic_offset[0]
    #     y_scenic = loc[1] + scenic_offset[1]
    #     return Vector(x_scenic, y_scenic, 0)
