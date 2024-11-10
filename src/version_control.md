# Version Control

The project adopts the **superproject** approach to manage a large
number of Git repositories. The F1EIGHTH repository itself, the said
superproject, saves the sub-projects as _Git submodules_ but does not
store their actual data. You can read the WikiBook
[here](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects)
to get the impression of superproject.

A Git submodule works as though it is a hyperlink within the mother
repository. The mother repository stores information about submodules
in the `.gitmodules` file. You can list them by:

```sh
git submodule status
```

> **Notice**
> 
> The official Autoware adopts a different version control strategy
> from ours. Do not confuse them.

## Download a Repository with Submodules

Always add the `--recursive` option when downloading a repository
containing submodules.

```sh
git clone --recursive https://github.com/NEWSLabNTU/F1EIGHTH.git
```

If you forget to add the option, the submodule directories will be
empty. You can get the submodule contents afterwards.

```sh
cd F1EIGHTH/
git submodule update --init --recursive
```

## Inspect a Submodule

Let's check the `src/autoware/core/autoware.core` submodule for
example. Open the `.gitmodules` and you can see the section below. It
tells the directory location to the submodule and also the upstream
URL.

```
[submodule "src/autoware/core/autoware.core"]
        path = src/autoware/core/autoware.core
        url = https://github.com/autowarefoundation/autoware.core.git

```

The path `src/autoware/core/autoware.core` is treated as a link file
in the viewpoint of the mother repo. It stores the commit hash to the
tracked Git repository. You can show the commit hash the command
below. If the commit hash changes, we go through the usual git add &
commit to save it.

```sh
$ git submodule status src/autoware/core/autoware.core
 99891401473b5740e640f5a0cc0412c0984b8e0b src/autoware/core/autoware.core (v1.0~1)
```

## Save Changes within a Submodule

To save the changes within a submodule, you must commit the changes
both in the submodule repo and in the mother repo in a two-step
fashion.

Let's see `src/autoware/sensor_kit/f1eighth_sensor_kit_launch`
submodule for example.

| Committed Changes                                             | Pushed to Upstream Repository                                                                            |
|---------------------------------------------------------------|----------------------------------------------------------------------------------------------------------|
| Changes within the `f1eighth_sensor_kit_launch` submodule.    | [f1eighth\_sensor\_kit_launch](https://github.com/NEWSLabNTU/f1eighth_sensor_kit_launch) subproject repo |
| New commit hash on the `f1eighth_sensor_kit_launch` submodule | [F1EIGHTH](https://github.com/NEWSLabNTU/F1EIGHTH) mother repo                                           |

The walk through goes like this.

```sh
# Go into the submodule and check to the branch we want to work on.
cd src/autoware/sensor_kit/f1eighth_sensor_kit_launch
git checkout main

# Do some work in the submodule.
touch a_new_file  # Create a file

# Commit and push to the upstream repo.
git add a_new_file
git commit -m 'Add a new file'
git push

# Go back to the mother repo
cd -

# Save the new commit hash on the submodule and push it to the upstream repo.
git add src/autoware/core/autoware.core
git commit -m 'Update the autoware.core submodule'
git push
```
