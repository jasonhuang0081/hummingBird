#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate


class Controller(ControllerBase):
    def init_control(self, param):
        pass

    def compute_control(self, param, state, reference, dt):
        left = 0.0
        right = 0.0

        left = saturate(left, 0, 1.0)
        right = saturate(right, 0, 1.0)

        return Command(left=left, right=right)


if __name__ == '__main__':
    c = Controller()
    c.run()
