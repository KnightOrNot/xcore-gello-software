from unittest.mock import Mock

import pytest

from xcore_gello_software.configuration import load_config


@pytest.mark.parametrize(
    "text",
    [
        "[read]\nhz = nan\n",
        "[read]\nsamples = 1.5\n",
        '[read]\ngripper_use_library_endpoints = "false"\n',
    ],
)
def test_invalid_read_defaults_rejected(tmp_path, text):
    path = tmp_path / "bad.toml"
    path.write_text(text)
    with pytest.raises(ValueError):
        load_config(path)


def test_read_uses_config_without_opening_real_serial(tmp_path, monkeypatch):
    from xcore_gello_software.commands import read

    path = tmp_path / "read.toml"
    path.write_text('[read]\ngello_port = "fake-port"\nsamples = 1\nhz = 5\n')
    monkeypatch.setenv("XCORE_GELLO_CONFIG", str(path))
    agent = Mock()
    agent.act.return_value = [0] * 7
    agent.get_gripper_state.return_value = None
    create = Mock(return_value=agent)
    monkeypatch.setattr(read, "GelloAgent", create)
    read.main(read.Args())
    create.assert_called_once_with(
        "fake-port", gripper_open_deg=None, gripper_close_deg=None
    )
    agent.act.assert_called_once()
