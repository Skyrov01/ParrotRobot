




script_robot_controls_tailwind = '''
<script>
window.addEventListener('DOMContentLoaded', () => {
    function togglePopup(popupId) {
        const popup = document.getElementById(popupId);
        if (popup) {
            popup.classList.toggle('show');
        }
    }

    document.getElementById('head_circle')?.addEventListener('click', () => togglePopup('head_popup'));
    document.getElementById('left_circle')?.addEventListener('click', () => togglePopup('left_popup'));
    document.getElementById('right_circle')?.addEventListener('click', () => togglePopup('right_popup'));
    document.getElementById('tilt_circle')?.addEventListener('click', () => togglePopup('tilt_popup'));
});

function updateServoStatus(id, angle, status) {
    const el = document.getElementById(id);
    if (!el) return;
    el.innerText = angle;

    const colors = {
        'ok': '#00bf63',      // green
        'moving': '#fbce37',  // yellow
        'error': '#e85642'    // red
    };
    el.style.backgroundColor = colors[status] || 'gray';
}
'''
