import re
import os
import sys
import time
import enum
import yaml
import copy
import signal
import typing
import hashlib
import traceback
import threading

import imviz as viz
import objtoolbox as otb

from .converter_ros2 import convert_config_to_ros2_launch
import launch


class Override:

    def __init__(self, kind=None, value=None, active=False):

        self.kind = kind
        self.value = value
        self.default_value = copy.deepcopy(value)
        self.active = active

        # try to generate a default value from kind

        if value is not None:
            return

        if kind is None:
            return

        try:
            self.value = kind()
            return
        except:
            pass

        if typing.get_origin(kind) is typing.Union:
            args = typing.get_args(kind)
            if len(args) == 2 and args[1] == type(None):
                try:
                    self.value = args[0]()
                    return
                except:
                    pass

        if type(kind) == enum.EnumMeta:
            self.value = list(kind.__members__.values())[0]
            return

        # TODO: initialize other types

    def __savestate__(self):

        d = self.__dict__.copy()
        del d["kind"]

        return d


class AduulmLaunchGui:

    def __init__(self, gen_config, params, initial_config=None):

        self.gen_config = gen_config
        self.params = params
        if initial_config is not None:
            self.initial_config = initial_config
        else:
            self.initial_config = LaunchConfig()
        self.config = copy.deepcopy(self.initial_config)

        self.launch_err_msg = ""

        self.overrides = {}
        self.filtered_overrides = {}
        self.filter_expr = ""

        # options

        self.clear_console = True
        self.reopen_after_launch = True
        self.show_all_overrides = False

        # generate save path

        m = hashlib.md5()
        m.update(sys.modules[gen_config.__module__].__file__.encode("utf8"))
        self.save_path = os.path.expanduser(
            f"~/.config/aduulm_launch_gui/{m.hexdigest()}")
        self.ini_path = os.path.join(self.save_path, "window_config.ini")

        # file path management

        self.file_path = os.getcwd()
        self.file_path_required = False
        self.file_path_action = None

        viz.set_main_window_title("aduulm_launch_gui")
        viz.set_ini_path(self.ini_path)
        viz.load_ini(self.ini_path)

        # initialize overrides

        self.update_overrides()
        otb.load(self, self.save_path)

        self.override_update_needed = False
        self.last_override_request_time = time.time()

        # temporary state

        self.finished = False

        # argument handling

        self.script = sys.argv[0]
        self.platform_name = re.search(r"platform_\w+", self.script).group(0)
        self.launch_file = os.path.basename(self.script)
        self.args = [a for a in sys.argv[1:] if a != "--gui"]

    def __savestate__(self):

        d = {}
        d["overrides"] = self.active_overrides()
        d["clear_console"] = self.clear_console
        d["reopen_after_launch"] = self.reopen_after_launch
        d["show_all_overrides"] = self.show_all_overrides
        d["file_path"] = self.file_path

        return d

    def start_launch_config(self):

        viz.hide_main_window()

        self.update_overrides()

        if self.clear_console:
            os.system("clear")

        launch_description = convert_config_to_ros2_launch(self.config)
        launch_service = launch.LaunchService(argv=sys.argv)
        launch_service.include_launch_description(launch_description)
        launch_service.run()

        viz.show_main_window()

    def active_overrides(self):

        return {k: v for k, v in self.overrides.items() if v.active}

    def insert_overrides(self, config):

        for k, v in self.active_overrides().items():
            segments = k.split(".")
            o = config.overrides()
            for s in segments[:-1]:
                o = getattr(o, s)
            setattr(o, segments[-1], v.value)

    def update_overrides(self):

        try:
            config = copy.deepcopy(self.initial_config)
            self.insert_overrides(config)
            self.gen_config(config, self.params)
            self.launch_err_msg = ""
            self.config = config
        except Exception as e:
            self.launch_err_msg = traceback.format_exc()

        new_overrides = {}
        for _, fields in self.config._getavail_overrides():
            for f in fields:
                val = f.default_value
                if f.value is not None:
                    val = f.value
                new_overrides[f.name] = Override(f.field_type, val, False)

        for k, v in new_overrides.items():
            if k not in self.overrides:
                continue
            o = self.overrides[k]
            v.active = o.active
            if type(v.value) == type(o.value):
                v.value = o.value

        self.overrides = new_overrides
        self.override_update_needed = False

        self.update_filter()

    def update_filter(self):

        if self.filter_expr == "":
            self.filtered_overrides = self.overrides
            return

        try:
            re_expr = re.compile(self.filter_expr)
            self.filtered_overrides = {k: v for k, v in self.overrides.items()
                                       if re_expr.search(k) is not None}
        except:
            pass

    def get_launch_command(self):

        cmd = "ros2 run " + self.platform_name + " " + self.launch_file
        for a in self.args:
            cmd += " " + a

        for name, o in self.active_overrides().items():
            cmd += f" {name}:={o.value}"

        return cmd

    def open_yaml_config(self, path):

        try:
            with open(path, "r") as fd:
                overs = yaml.safe_load(fd)
        except:
            traceback.print_exc()
            return

        for name, val in overs.items():
            if name not in self.overrides:
                continue
            self.overrides[name].value = val
            self.overrides[name].active = True

    def save_yaml_config(self, path):

        res = {name: o.value
               for name, o in self.active_overrides().items()}

        with open(path, "w+") as fd:
            yaml.dump(res, fd)

    def render_toolbar(self):

        if viz.begin_table("toolbar", 2, viz.TableFlags.SIZING_STRETCH_PROP):

            viz.table_setup_column("search", init_width_or_weight=0.9)
            viz.table_setup_column("launch", init_width_or_weight=0.15)
            viz.table_next_row()
            viz.table_next_column()

            if viz.button(f"{viz.Icon.FOLDER}"):
                self.defer_file_action(self.open_yaml_config)
            if viz.is_item_hovered():
                viz.begin_tooltip()
                viz.text("Import from YAML")
                viz.end_tooltip()

            viz.same_line()

            if viz.button(f"{viz.Icon.FLOPPY_DISK}"):
                self.defer_file_action(self.save_yaml_config)
            if viz.is_item_hovered():
                viz.begin_tooltip()
                viz.text("Export to YAML")
                viz.end_tooltip()

            viz.same_line()

            if viz.button(f"{viz.Icon.COPY}"):
                viz.set_clipboard(self.get_launch_command())
            if viz.is_item_hovered():
                viz.begin_tooltip()
                viz.text("Copy launch command to clipboard")
                viz.end_tooltip()

            viz.same_line()

            if viz.button(f"{viz.Icon.GEAR}"):
                viz.open_popup("options_popup")
            if viz.is_item_hovered():
                viz.begin_tooltip()
                viz.text("Settings")
                viz.end_tooltip()

            viz.push_mod_any()

            if viz.begin_popup("options_popup"):
                if viz.menu_item("Clear console", selected=self.clear_console):
                    viz.set_mod(True)
                    self.reopen_after_launch = not self.reopen_after_launch
                if viz.menu_item("Reopen after launch", selected=self.reopen_after_launch):
                    viz.set_mod(True)
                    self.reopen_after_launch = not self.reopen_after_launch
                if viz.menu_item("Show all overrides", selected=self.show_all_overrides):
                    viz.set_mod(True)
                    self.show_all_overrides = not self.show_all_overrides
                viz.end_popup()

            if viz.pop_mod_any():
                otb.save(self, self.save_path)

            viz.same_line()

            self.filter_expr = viz.input("regex search", self.filter_expr)
            if viz.mod():
                self.update_filter()

            viz.table_next_column()

            if self.launch_err_msg == "":
                if viz.button("launch config"):
                    if self.reopen_after_launch:
                        self.start_launch_config()
                    else:
                        self.finished = True
            else:
                viz.text("config error", color=(1.0, 0.0, 0.0))
                if viz.is_item_hovered():
                    viz.begin_tooltip()
                    viz.text(self.launch_err_msg)
                    viz.end_tooltip()

            viz.end_table()

            if self.launch_err_msg != "":
                viz.separator()
                viz.text(self.launch_err_msg)

    def render_override_value(self, name, ovr):

        viz.push_mod_any()

        if ovr.value == None:
            viz.text(f"unsupported {ovr.kind}")
        elif type(ovr.kind) == enum.EnumMeta:
            keys = list(ovr.kind.__members__.keys())
            vals = list(ovr.kind.__members__.values())
            idx = vals.index(ovr.value)
            idx = viz.combo(f"###{name}_value", keys, vals.index(ovr.value))
            ovr.value = vals[idx]
        else:
            ovr.value = viz.autogui(ovr.value, f"###{name}_value")

        if viz.pop_mod_any():
            ovr.active = True
            viz.set_mod(True)

    def render_overrides(self):

        viz.push_mod_any()

        for name, ovr in self.filtered_overrides.items():
            if self.filter_expr == "" and not self.show_all_overrides and not ovr.active:
                continue
            viz.table_next_column()
            if ovr.active:
                color = (0.4, 1.0, 0.4)
            elif ovr.value == None:
                color = (0.5, 0.5, 0.5)
            else:
                color = (1.0, 1.0, 1.0)
            viz.text(name, color)
            viz.table_next_column()
            self.render_override_value(name, ovr)
            viz.table_next_column()
            viz.begin_disabled(ovr.value == None)
            ovr.active = viz.checkbox(f"###{name}_override", ovr.active)
            viz.end_disabled()
            viz.table_next_row()

        if viz.pop_mod_any():
            self.override_update_needed = True
            self.last_override_request_time = time.time()

        if (self.override_update_needed
                and time.time() - self.last_override_request_time > 0.5):
            self.update_overrides()
            otb.save(self, self.save_path)

    def defer_file_action(self, action):

        self.file_path_required = True
        self.file_path_action = action

    def render_file_dialog(self):

        if self.file_path_required:
            viz.open_popup("Select path")

        self.file_path = viz.file_dialog_popup("Select path", self.file_path)
        if viz.mod():
            self.file_path_action(self.file_path)
            otb.save(self, self.save_path)

        self.file_path_required = False

    def render(self):

        if viz.begin_window("overrides",
                            position=(0.0, 0.0),
                            size=viz.get_main_window_size(),
                            title_bar=False,
                            resize=False,
                            move=False):

            self.render_toolbar()
            viz.separator()

            if viz.begin_child("scroll_overrides"):

                tbl_flags = viz.TableFlags.RESIZABLE | viz.TableFlags.SIZING_STRETCH_PROP
                if viz.begin_table("overrides", 3, tbl_flags):
                    viz.table_setup_column("Name", init_width_or_weight=0.6)
                    viz.table_setup_column("Value", init_width_or_weight=0.3)
                    viz.table_setup_column(
                        "Override", init_width_or_weight=0.1)
                    viz.table_headers_row()
                    viz.table_next_row()

                    self.render_overrides()

                    viz.end_table()
                viz.end_child()
            viz.end_window()

        self.render_file_dialog()

    def gen_config_with_overrides(self):

        try:
            while viz.wait(vsync=True, powersave=True) and not self.finished:
                self.render()
        except KeyboardInterrupt:
            pass

        if not self.finished:
            sys.exit(0)

        if self.clear_console:
            os.system("clear")

        self.update_overrides()

        viz.hide_main_window()

        return self.config
