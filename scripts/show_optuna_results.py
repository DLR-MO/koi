#!/usr/bin/env python3

import argparse
import optuna
from optuna.visualization import (
    plot_optimization_history,
    plot_param_importances,
    plot_pareto_front,
    plot_rank,
    plot_terminator_improvement
)

parser = argparse.ArgumentParser()
parser.add_argument('-j', '--journal-file', type=str, required=True)
parser.add_argument('-s', '--study-name', type=str, default='reach_study')
args = parser.parse_args()

storage = optuna.storages.JournalStorage(optuna.storages.JournalFileStorage(args.journal_file))
study = optuna.load_study(study_name=args.study_name, storage=storage)
fig = plot_optimization_history(study)
fig.show()
input('Press enter to continue to the next plot.')
fig = plot_param_importances(study)
fig.show()
input('Press enter to continue to the next plot.')
if len(study.directions) > 1:
    fig = plot_pareto_front(study)
    fig.show()
    input('Press enter to continue to the next plot.')
fig = plot_rank(study)
fig.show()
input('Press enter to continue to the next plot.')
try:
    import torch
except ImportError:
    print('Could not import torch. Skipping plot_terminator_improvement.')
else:
    fig = plot_terminator_improvement(study)
    fig.show()
