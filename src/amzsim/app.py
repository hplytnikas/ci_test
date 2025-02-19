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

from amzsim_frontend.amzsim_presenter.presenter import Presenter

if __name__ == "__main__":
    app = Presenter()
    app.start_app()
