# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE: Import here your extension examples to be propagated to ISAAC SIM Extensions startup
from .franka_rope import IsaacUIUtils, VRUIUtils

# IsaacUIUtils.setUp()
VRUIUtils.setUp()

# import os
# from omni.isaac.examples.base_sample import BaseSampleExtension

# class FrankaRopeExtension(BaseSampleExtension):
#     def on_startup(self, ext_id: str):
#         super().on_startup(ext_id)
#         super().start_extension(
#             menu_name="",
#             submenu_name="",
#             name="Franka Rope Ext",
#             title="Franka Rope Example",
#             doc_link="",
#             overview="",
#             file_path=os.path.abspath(__file__),
#             sample=IsaacUIUtils.setUp(),
#         )
#         return

