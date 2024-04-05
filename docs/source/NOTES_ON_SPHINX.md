# Documentation with Sphinx

## Installation of Sphinx
```
sudo apt-get install python3-sphinx -y

sudo pip install sphinx_rtd_theme myst_parser docutils==0.20

# sudo pip install sphinx sphinx_rtd_theme myst_parser m2r2 setuptools
```

## Build HTML

```
cd docs
sphinx-build source build/html
```

## Ignore External Modules

Add the following to the conf.py file

autodoc_mock_imports = ['rospy', 'commander_moveit', 'geometry_msgs', 'tf']


### Deploy Sphinx Generated HTML: Best Practice

1. Define workflow to build sphinx html files when pushed. Make sure the command to run is `cd docs && sphinx-build source build/html`. The output is saved to the folder `docs/build/html`
2. Add index.html under the `docs` folder to redirect to the generated index.html file.
```
<meta http-equiv="refresh" content="0; URL=build/html/index.html" />
```
3. The workflow should include deployment of the `docs` folder to a new branch `gh-pages`, according to the following in the workflow yaml file.
```
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        if: ${{ github.event_name == 'push' && github.ref == 'refs/heads/main' }}
        with:
          publish_branch: gh-pages
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: docs
          force_orphan: true
```

4. Under the Github repository Settings, Github Pages, select Deploy from Branch `gh-pages`, and deploy from the folder `docs`.


### The Underscore

The underscore folder names are ignored by the Jekyll engine in Github Page deployment.  Add the file `.nojekyll` in the same directory as index.html to not using Jekyll.

The underscore folder names are used by Sphinx under html including `_static` and `source`.


## M2R2: Handling Markdown in Sphinx
```
sudo pip install m2r2
```

## References
[Deploy Sphinx documentation to Github Pages](https://coderefinery.github.io/documentation/gh_workflow/)

[Include Readme](https://stackoverflow.com/questions/46278683/include-my-markdown-readme-into-sphinx/68005314#68005314)

[Starter Workflow Examples](https://github.com/actions/starter-workflows/)