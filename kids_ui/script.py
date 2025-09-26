




script_robot_controls_tailwind = '''
<script>
window.addEventListener('DOMContentLoaded', () => {
    function togglePopup(popupId) {
        const popup = document.getElementById(popupId);
        if (popup) {
            popup.classList.toggle('show');
        }
    }

    document.getElementById('servo_head_rotate')?.addEventListener('click', () => togglePopup('head_popup'));
    document.getElementById('servo_left_wing')?.addEventListener('click', () => togglePopup('left_popup'));
    document.getElementById('servo_right_wing')?.addEventListener('click', () => togglePopup('right_popup'));
    document.getElementById('servo_head_tilt')?.addEventListener('click', () => togglePopup('tilt_popup'));
});
</script>

'''
script_status_servo = '''
<script>
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

(function(){
  const map = {
    "head_tilt": "servo_head_tilt",
    "left_wing": "servo_left_wing",
    "right_wing": "servo_right_wing",
    "head_rotate": "servo_head_rotate"
  };

  function color(status){
    if (status === 3) return "error";   // ERROR
    if (status === 1) return "moving";  // MOVING
    return "ok";                        // IDLE/HOLD -> ok
  }

  function applySnapshot(snap){
    if (!snap) return;
    Object.values(snap).forEach(s => {
      const id = map[s.target];
      if (id) updateServoStatus(id, s.commanded_deg, color(s.status));
    });
  }

  function connect(){
    const es = new EventSource("http://192.168.8.120:5000/servo/stream");
    es.onmessage = (evt) => {
      const data = JSON.parse(evt.data);
      if (data.snapshot) {
        return applySnapshot(data.snapshot);
      }
      if (data.target) {
        const id = map[data.target];
        if (id) updateServoStatus(id, data.commanded_deg, color(data.status));
      }
      if (data.event) {
        // handle custom UI events pushed by backend
        console.log("UI event:", data.event, data);
      }
    };
    es.onerror = () => { try { es.close(); } catch(e){} setTimeout(connect, 1500); };
  }

  window.addEventListener("DOMContentLoaded", connect);
})();
</script>

'''
