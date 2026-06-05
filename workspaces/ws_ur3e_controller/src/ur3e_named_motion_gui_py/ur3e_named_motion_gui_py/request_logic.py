from dataclasses import dataclass

from ur3e_controller_msgs.srv import ExecuteNamedTarget


@dataclass(frozen=True)
class ResponseSummary:
    success: bool
    text: str


def build_execute_request(target_name: str, human_confirmation: str = ''):
    request = ExecuteNamedTarget.Request()
    request.target_name = target_name
    request.execute = True
    request.human_confirmation = human_confirmation
    return request


def summarize_response(response) -> ResponseSummary:
    success = response.status == 'executed' and response.executed
    outcome = 'SUCCESS' if success else 'FAILED'
    text = (
        f'{outcome}: accepted={response.accepted} planned={response.planned} '
        f'executed={response.executed} status={response.status} '
        f'message={response.message}'
    )
    return ResponseSummary(success=success, text=text)
