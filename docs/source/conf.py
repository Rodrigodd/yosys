#!/usr/bin/env python3
import sys
import os

project = 'YosysHQ Yosys'
author = 'YosysHQ GmbH'
copyright ='2022 YosysHQ GmbH'

# select HTML theme
html_theme = 'furo'
templates_path = ["_templates"]
html_logo = '../static/logo.png'
html_favicon = '../static/favico.png'
html_css_files = ['yosyshq.css', 'custom.css']

html_theme_options = {
    "sidebar_hide_name": True,

    "light_css_variables": {
        "color-brand-primary": "#d6368f",
        "color-brand-content": "#4b72b8",
        "color-api-name": "#8857a3",
        "color-api-pre-name": "#4b72b8",
        "color-link": "#8857a3",
    },

    "dark_css_variables": {
        "color-brand-primary": "#e488bb",
        "color-brand-content": "#98bdff",
        "color-api-name": "#8857a3",
        "color-api-pre-name": "#4b72b8",
        "color-link": "#be95d5",
    },
}

# These folders are copied to the documentation's HTML output
html_static_path = ['../static', "../images"]

# code blocks style 
pygments_style = 'colorful'
highlight_language = 'none'

extensions = ['sphinx.ext.autosectionlabel', 'sphinxcontrib.bibtex']

# Ensure that autosectionlabel will produce unique names
autosectionlabel_prefix_document = True
autosectionlabel_maxdepth = 1

# assign figure numbers
numfig = True

bibtex_bibfiles = ['literature.bib']

latex_elements = {
        'preamble': r'''
\usepackage{lmodern}
\usepackage{comment}

'''
}

def setup(sphinx):
	sys.path += [os.path.dirname(__file__) + "/../util"]
	from RtlilLexer import RtlilLexer
	sphinx.add_lexer("RTLIL", RtlilLexer)

	from YoscryptLexer import YoscryptLexer
	sphinx.add_lexer("yoscrypt", YoscryptLexer)