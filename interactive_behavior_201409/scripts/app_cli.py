#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import click
from pathlib2 import Path
import rospkg
import shutil
import xml.etree.ElementTree as ET
import yaml


DEF_PKG = "interactive_behavior_201409"
ROSPKG = rospkg.RosPack()


def package_path(name):
    return Path(ROSPKG.get_path(name))


def current_package(curdir=None):
    if curdir is None:
        curdir = Path(".").absolute()
    else:
        curdir = Path(curdir).absolute()
    while curdir.parent != curdir:
        click.echo(curdir)
        for p in curdir.iterdir():
            if p.name == "package.xml":
                with p.open() as f:
                    xml = ET.fromstring(f.read())
                    try:
                        return xml.find("name").text
                    except Exception as e:
                        click.echo(e)
                        return DEF_PKG
        curdir = curdir.parent
    return DEF_PKG


CUR_PKG = current_package()


def list_apps(package):
    pkgdir = package_path(package)
    appdir = pkgdir / "apps"
    appfiles = appdir.glob("**/*.app")
    apps = []
    for appfile in appfiles:
        with open(str(appfile)) as f:
            dic = yaml.load(f)
            dic["name"] = "/".join([package, appfile.stem])
            apps.append(dic)
    return apps


def yaml_dump(dic, f):
    yaml.safe_dump(dic, f,
                   encoding="utf-8",
                   allow_unicode=True,
                   default_flow_style=False)


@click.group()
def cli():
    pass


@cli.command()
@click.option("--package", type=str, required=True, prompt="Name of package", default=CUR_PKG)
def list(package):
    apps = list_apps(package)
    if not apps:
        click.echo("No app found")
        return
    click.echo()
    click.echo("Apps in package '%s':" % package)
    for app in apps:
        click.echo("\t%s (%s)" % (app["name"], app["display"]))


@cli.command()
@click.option("--package", type=str, required=True, prompt="Name of package", default=CUR_PKG)
@click.option("--name", type=str, required=True, prompt="Name of application")
@click.option("--display", type=str, required=True, prompt="Display name")
@click.option("--description", type=str, required=True, prompt="Description")
@click.option("--platform", type=str, required=True, prompt="Platform", default="pr2")
@click.option("--icon", type=str, required=True, prompt="Path to icon image", default="default")
def add(package, name, display, description, platform, icon):
    pkgdir = package_path(package)
    app_root_dir = pkgdir / "apps"
    if not app_root_dir.is_dir():
        app_root_dir.mkdir(parents=True, exist_ok=True)
    apps = [a["name"] for a in list_apps(package)]
    if "/".join([package, name]) in apps:
        raise click.BadParameter("App already exists")
    appdir = app_root_dir / name

    appdir.mkdir(parents=True, exist_ok=True)
    base_file = appdir / name
    # icon
    if icon == "default":
        icon = package_path("interactive_behavior_201409") / "data" / "default_icon.png"
        icon_ext = Path(icon).suffix
    else:
        icon_ext = ".png"
    icon_file = base_file.with_suffix(icon_ext)
    shutil.copyfile(str(icon), str(icon_file))
    # launch
    launch_file = base_file.with_suffix(".xml")
    with launch_file.open("w") as f:
        f.write(u"""\
<launch>
  <node name="run_{name}" pkg="{package}" type="{name}.l" output="screen" required="true"/>
</launch>
""".format(package=package, name=name))
    # interface
    interface_file = base_file.with_suffix(".interface")
    with interface_file.open("w") as f:
        f.write(u"""\
published_topics: {}
subscribed_topics: {}
""")
    # node
    node_file = base_file.with_suffix(".l")
    with node_file.open("w") as f:
        f.write(u"""\
#!/usr/bin/env roseus
;; {name}.l

(ros::roseus "run_{name}")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(defun main ()
  ;; TODO: implement app
  t)


(if (main) (exit 0) (exit 1))
""".format(name=name))
    node_file.chmod(0755)
    # app
    app_dic = {
        "display": display,
        "description": description,
        "platform": platform,
        "launch": package + "/" + launch_file.name,
        "interface": package + "/" + interface_file.name,
        "icon": package + "/" + icon_file.name,
    }
    app_file = base_file.with_suffix(".app")
    with app_file.open("w") as f:
        yaml_dump(app_dic, f)

    # add to installed file
    installed_file = next(app_root_dir.glob("*.installed"), None)
    if installed_file is None:
        installed_file = app_root_dir / "app.installed"
        with installed_file.open("w") as f:
            yaml_dump({"apps": []}, f)
        click.echo("installed file not found, creating new installed file: %s" % installed_file)
    else:
        click.echo("found installed file: %s" % installed_file)

    with installed_file.open("r") as f:
        installed_dic = yaml.load(f)
    installed_dic["apps"].append({
        "app": package + "/" + name,
        "display": display,
    })
    with installed_file.open("w") as f:
        yaml_dump(installed_dic, f)

    click.echo("Application '%s/%s' is created." % (package, name))
    click.echo("Go to %s" % appdir.relative_to(Path(".").absolute()))


@cli.command()
@click.option("--package", type=str, required=True, prompt="Name of package", default=CUR_PKG)
@click.option("--name", type=str, required=True, prompt="Name of application")
def remove(package, name):
    pkgdir = package_path(package)
    app_root_dir = pkgdir / "apps"
    if not app_root_dir.is_dir():
        app_root_dir.mkdir(parents=True, exist_ok=True)
    apps = [a["name"] for a in list_apps(package)]
    if "/".join([package, name]) not in apps:
        raise click.BadParameter("App not found")
    appdir = app_root_dir / name
    click.echo("Removing %s/%s" % (package, name))
    if not click.confirm("Are you sure to continue?"):
        raise click.Abort()

    shutil.rmtree(str(appdir))

    installed_file = next(app_root_dir.glob("*.installed"), None)
    if installed_file is None:
        click.echo("installed file not found. skipping")
    else:
        with installed_file.open("r") as f:
            dic = yaml.load(f)
        newdic = {"apps": []}
        for app in dic["apps"]:
            if app["app"] != package + "/" + name:
                newdic["apps"].append(app)
        with installed_file.open("w") as f:
            yaml_dump(newdic, f)

    click.echo("Done!")

@cli.command()
@click.option("--package", type=str, required=True, prompt="Name of package", default=CUR_PKG)
@click.option("--name", type=str, required=True, prompt="Name of application")
def rename(package, name):
    click.echo(package)
    click.echo(name)



if __name__ == '__main__':
    cli()
