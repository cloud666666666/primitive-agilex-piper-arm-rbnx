# SPDX-License-Identifier: Apache-2.0
"""Shared Robonix provider for the selectable Piper backends."""

from robonix_api import Primitive


# RBNX_INSTANCE_NAME overrides this default for deployment instances such as
# left_piper and right_piper. Both backends must register through this single
# provider so the package has one lifecycle and one capability identity.
piper_arm = Primitive(
    id="piper_ctl",
    namespace="robonix/primitive/arm",
)
