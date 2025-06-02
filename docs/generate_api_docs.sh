#!/bin/bash

sphinx-apidoc -M -H "RoBits API" \
--ext-autodoc \
--ext-intersphinx \
--ext-viewcode \
--no-toc \
--implicit-namespaces \
-o source/api ../robits
