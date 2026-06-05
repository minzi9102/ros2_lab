from types import SimpleNamespace

from ur3e_named_motion_gui_py.request_logic import (
    build_execute_request,
    summarize_response,
)


def make_response(status, *, executed=False, accepted=False, planned=False):
    return SimpleNamespace(
        accepted=accepted,
        planned=planned,
        executed=executed,
        status=status,
        message=f'{status} message',
    )


def test_home_request_fields_are_fixed_for_sim_execute():
    request = build_execute_request('home')

    assert request.target_name == 'home'
    assert request.execute
    assert request.human_confirmation == ''


def test_ready_request_fields_are_fixed_for_sim_execute():
    request = build_execute_request('ready')

    assert request.target_name == 'ready'
    assert request.execute
    assert request.human_confirmation == ''


def test_request_can_carry_optional_confirmation_token():
    request = build_execute_request('home', 'token')

    assert request.human_confirmation == 'token'


def test_executed_response_is_success():
    summary = summarize_response(
        make_response('executed', accepted=True, planned=True, executed=True)
    )

    assert summary.success
    assert 'SUCCESS' in summary.text
    assert 'status=executed' in summary.text


def test_rejected_response_preserves_status_and_message():
    summary = summarize_response(make_response('rejected_real_gate'))

    assert not summary.success
    assert 'FAILED' in summary.text
    assert 'status=rejected_real_gate' in summary.text
    assert 'rejected_real_gate message' in summary.text


def test_planning_failed_response_is_failure():
    summary = summarize_response(make_response('planning_failed'))

    assert not summary.success
    assert 'status=planning_failed' in summary.text


def test_execution_failed_response_is_failure():
    summary = summarize_response(
        make_response('execution_failed', accepted=True, planned=True)
    )

    assert not summary.success
    assert 'status=execution_failed' in summary.text
