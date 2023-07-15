# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

import os
import sys
import sphinx_rtd_theme

sys.path.append(os.path.abspath('exts'))

# -- Project information -----------------------------------------------------

project = 'asclinic-wiki'
copyright = '2021, The University of Melbourne'
author = 'Paul Beuchat'

# The full version, including alpha/beta/rc tags
release = '0.1.0'



# -- Useful Links ------------------------
# > For setup of C++ auto documentation
#   https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/


# -- General configuration ---------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
	#'sphinx_comments',
	#
	# > The the theme:
	'sphinx_rtd_theme',
	#'sphinx_rtd_dark_mode',
	#'sphinxcontrib.examplecode',
	#'sphinxcontrib.osexample',
	#
	# > For graphics
	'sphinx.ext.graphviz',
	#
	# > For auto documentation:
	#'sphinx.ext.napoleon',
]

# If using the "sphinx_rtd_dark_mode" extension,
# then specify the default starting mode:
# > user starts in dark mode
#default_dark_mode = True
# > user starts in light mode
#default_dark_mode = False

# Configuration for the sphinx_comments extension
#comments_config = {
#   "hypothesis": True
#}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']




# -- Napoleon configuration --------------
#    > For python auto documentation
#napoleon_google_docstring = False
#napoleon_numpy_docstring = True
#napoleon_include_init_with_doc = False
#napoleon_include_private_with_doc = True
#napoleon_include_special_with_doc = True
#napoleon_use_admonition_for_examples = False
#napoleon_use_admonition_for_notes = False
#napoleon_use_admonition_for_references = False
#napoleon_use_ivar = False
#napoleon_use_param = True
#napoleon_use_rtype = True
#napoleon_preprocess_types = False
#napoleon_type_aliases = None
#napoleon_attr_annotations = True



# -- GraphViz configuration --------------
graphviz_output_format = 'svg'


# -- Options for HTML output -------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
#html_theme_path = ['_themes', ]
html_theme_options = {
	# ToC options
	'collapse_navigation': False,
	'sticky_navigation': True,
	'navigation_depth': 4,
	'includehidden': True,
	'titles_only': False,
	# Misc options
	#'analytics_id': 'UA-XXXXXXX-1',  #  Provided by Google in your dashboard
	#'analytics_anonymize_ip': False,
	'display_version': True,
	'logo_only': False,
	'prev_next_buttons_location': 'both', # Options: bottom | top | both | None
	'style_external_links': True,
	#'vcs_pageview_mode': '', # Options: blob | edit | raw
	'style_nav_header_background': '#2980B9',
	# File-wide metadata
	#'gitlab_url': 'https://gitlab.ethz.ch/dfall/dfall-system'
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# These paths are either relative to html_static_path
# or fully qualified paths (eg. https://...)
html_css_files = [
    #'css/dark.css',
    'css/custom_table_behaviour.css',
]

# NOTES FOR CUSTOM CSS:
# > The custom table behaviour is based on the following posts:
#   - https://stackoverflow.com/questions/69359978/grid-table-does-not-word-wrap
#   - https://stackoverflow.com/questions/68182085/word-wrap-in-sphinx-autosummary-generated-table



# If true, "(C) Copyright ..." is shown in the HTML footer. Default is True.
html_show_copyright = False
