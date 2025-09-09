# --- Styles ---
style_parrot_control = '''
<style>
.parrot-container {
    position: relative;
    width: 100%;
    max-width: 900px;
    margin: auto;
}
.parrot-image {
    width: 100%;
}
.hotspot {
    position: absolute;
    width: 30px;
    height: 30px;
    background-color: gold;
    border: 3px solid white;
    border-radius: 50%;
    cursor: pointer;
    transition: transform 0.2s;
}
.hotspot:hover {
    transform: scale(2.5);
}
.popup {
    position: absolute;
    padding: 10px;
    background: white;
    border: 2px solid gold;
    border-radius: 10px;
    display: none;
    width: 200px;
    z-index: 10;
}
</style>
'''


style_robot_controls_tailwind = '''
<style type="text/tailwindcss">
    @layer components {
        .hotspot {
            @apply bg-yellow-400 w-8 h-8 rounded-full border-4 border-white absolute cursor-pointer transition-transform duration-150;
        }
        .hotspot:hover {
            @apply scale-150;
        }
        .popup {
            @apply absolute p-4 bg-white border-yellow-500 border-2 rounded-lg shadow-xl w-56 z-10 hidden;
        }
        .popup.show {
            @apply block;
        }
    }
</style>
'''


style_navbar = """
<style>
.nav-home {
    position: relative;
    background-color: white;
    color: #2F2F2F;
    transition: all 0.3s ease-in-out;
}

.nav-home:hover {
    background-image: url('/static/images/namibia.png');
    background-size: cover;
    background-position: center;
    color: white; /* Text turns white on flag */
}
</style>
"""