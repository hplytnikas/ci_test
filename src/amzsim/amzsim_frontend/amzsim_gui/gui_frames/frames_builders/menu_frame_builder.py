#!/usr/bin/env python3

# AMZ Driverless Project
#
# Copyright (c) 2023 Authors:
# - Bartosz Mila <bamila@student.ethz.ch>
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

# pylint: disable= import-error
import os
import customtkinter
from PIL import Image


class MenuFrameBuilder:
    """
    Class that is used to create and instantiate MenuFrame components, such as checkboxes,
    optionmenu etc. and puts them in correct places.

    Attributes
    ----------
    menu_frame : MenuFrame
        Class MenuFrame for which components will be created.
    """

    def __init__(self, menu_frame):
        self.menu_frame = menu_frame

    def build_menu_frame(self):
        """
        Builds all of the frame components.
        """
        self.__configure_menu_frame()
        self.__load_menu_frame_images()
        self.__setup_menu_frame()

    def __configure_menu_frame(self):
        """
        Configures menu frame object and its grid.
        """
        self.menu_frame.configure(corner_radius=0)
        self.menu_frame.grid(row=0, column=0, sticky="nsew")
        self.menu_frame.grid_rowconfigure(4, weight=1)

    def __load_menu_frame_images(self):
        """
        Creates image components used in menu frame.
        """
        image_path = os.path.join(
            os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            ),
            "images",
        )
        self.menu_frame.img_sim = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "amzsim_t.png")), size=(240, 144)
        )
        self.menu_frame.img_home_light = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "home_light.png")), size=(20, 20)
        )
        self.menu_frame.img_home_dark = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "home_dark.png")), size=(20, 20)
        )
        self.menu_frame.img_config_light = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "config_light.png")), size=(20, 20)
        )
        self.menu_frame.img_config_dark = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "config_dark.png")), size=(20, 20)
        )

    def __setup_menu_frame(self):
        """
        Setups the rest of menu frame components.
        """

        # Create label and loads amzsim logo image into it.
        self.menu_frame.label_amzsim_img = customtkinter.CTkLabel(
            self.menu_frame, image=self.menu_frame.img_sim, text="", compound="left"
        )
        self.menu_frame.label_amzsim_img.grid(row=0, column=0, padx=20, pady=30)

        # Create buttons home, config, and vehicle which change frames on the right.
        self.menu_frame.button_home = customtkinter.CTkButton(
            self.menu_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Home",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            anchor="w",
            image=self.menu_frame.img_home_dark,
        )
        self.menu_frame.button_home.grid(row=1, column=0, sticky="ew")
        self.menu_frame.button_config = customtkinter.CTkButton(
            self.menu_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Configuration",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            anchor="w",
            image=self.menu_frame.img_config_dark,
        )
        self.menu_frame.button_config.grid(row=2, column=0, sticky="ew")
        # self.menu_frame.button_vehicle = customtkinter.CTkButton(
        #     self.menu_frame,
        #     corner_radius=0,
        #     height=40,
        #     border_spacing=10,
        #     text="Vehicle",
        #     fg_color="transparent",
        #     text_color=("gray10", "gray90"),
        #     hover_color=("gray70", "gray30"),
        #     anchor="w",
        #     image=self.menu_frame.img_home_dark,
        # )
        # self.menu_frame.button_vehicle.grid(row=3, column=0, sticky="ew")

        # Create optionmenu for changing aplication theme
        self.menu_frame.appearance_mode_menu = customtkinter.CTkOptionMenu(
            self.menu_frame, values=["Dark", "Light"]
        )
        self.menu_frame.appearance_mode_menu.grid(
            row=6, column=0, padx=20, pady=20, sticky="s"
        )
