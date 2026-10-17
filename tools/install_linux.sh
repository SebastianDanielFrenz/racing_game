#!/usr/bin/env bash
# Self-contained native installer/updater. No root writes unless --install-deps.
set -euo pipefail
ORIGINAL_ARGS=("$@")
REPO=https://github.com/SebastianDanielFrenz/racing_game.git
INSTALL_DIR=${RG_INSTALL_DIR:-"$HOME/racing_game"}
MAP_DIR=${RG_G2M_CACHE:-"$HOME/geo2map_cache"}
MODE=vr
JOBS=${RG_BUILD_JOBS:-4}
STEAM_USER=
STEAM_ONLY=0
NO_STEAM=0
INSTALL_DEPS=0
usage() {
    cat <<'HELP'
Install/update GitHub main natively on Linux ARM64, under ~/racing_game.
Usage: bash install_linux.sh [options]
  --install-dir PATH   Dedicated checkout, must be inside your home
  --map-dir PATH       Existing geo2map stores (default ~/geo2map_cache)
  --desktop            Flat-screen launcher (default: OpenXR VR)
  --vr                 OpenXR launcher
  --jobs N             Compile workers (default 4; reduce on low-memory devices)
  --install-deps       Install build tools using apt/pacman and sudo
  --steam-user ID      Steam userdata account directory, for multiple accounts
  --steam-only         Retry Steam registration/artwork without updating/building
  --no-steam           Install/update without a Steam shortcut
  --help               This help
Steam must be fully exited for safe shortcut registration. It is never killed.
Maps are NOT on GitHub: copy your geo2map stores separately, or use Flat World.
HELP
}
die() { echo "ERROR: $*" >&2; exit 1; }
while (($#)); do
    case "$1" in
        --install-dir|--map-dir|--jobs|--steam-user)
            (($# >= 2)) || die "Missing value after $1"
            case "$1" in
                --install-dir) INSTALL_DIR=$2;; --map-dir) MAP_DIR=$2;;
                --jobs) JOBS=$2;; --steam-user) STEAM_USER=$2;;
            esac; shift 2;;
        --desktop) MODE=desktop; shift;; --vr) MODE=vr; shift;;
        --steam-only) STEAM_ONLY=1; shift;; --no-steam) NO_STEAM=1; shift;;
        --install-deps) INSTALL_DEPS=1; shift;;
        --help|-h) usage; exit 0;; *) die "Unknown option: $1";;
    esac
done
[[ $(uname -s) == Linux ]] || die "Run this on the Linux device, not Windows."
[[ $(uname -m) == aarch64 || $(uname -m) == arm64 ]] || die "This installer requires native Linux ARM64."
[[ $JOBS =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer"
[[ $EUID != 0 ]] || die "Run as your normal user, not root; --install-deps uses sudo separately."
if ((INSTALL_DEPS)); then
    if command -v steamos-readonly >/dev/null && [[ $(steamos-readonly status) == enabled ]]; then
        die "SteamOS root is read-only. Provide build tools through a development environment; this script will not unlock the OS."
    elif command -v apt-get >/dev/null; then
        sudo apt-get update
        sudo apt-get install -y git git-lfs build-essential cmake ninja-build curl python3 unzip pkg-config
    elif command -v pacman >/dev/null; then
        sudo pacman -S --needed git git-lfs base-devel cmake ninja curl python unzip pkgconf
    else
        die "Install Git/LFS, a C++20 compiler, CMake >=3.25, Ninja, curl, Python3, unzip and flock using your distro."
    fi
fi
for program in git python3 flock; do command -v "$program" >/dev/null || die "Missing $program (see --install-deps)"; done
INSTALL_DIR=$(python3 - "$INSTALL_DIR" <<'PY'
import pathlib,sys
home=pathlib.Path.home().resolve(); target=pathlib.Path(sys.argv[1]).expanduser().resolve()
if not target.is_relative_to(home) or target == home:
    raise SystemExit('Install directory must be a subdirectory of your home')
print(target)
PY
)
STATE_DIR="$INSTALL_DIR/.local-install"
mkdir -p "$STATE_DIR"
# Updating main may replace this tracked script. Execute a private snapshot so
# Bash never reads a half-updated installer from the checkout during git merge.
if [[ ${RG_INSTALL_SNAPSHOT:-0} != 1 ]]; then
    [[ -f ${BASH_SOURCE[0]:-} ]] || die "Download this script to a file before running it."
    snapshot=$(mktemp "$STATE_DIR/installer-run.XXXXXX.sh")
    cp -- "${BASH_SOURCE[0]}" "$snapshot"
    exec env RG_INSTALL_SNAPSHOT=1 bash "$snapshot" "${ORIGINAL_ARGS[@]}"
fi
exec 9>"$STATE_DIR/install.lock"
flock -n 9 || die "Another installer is already running for $INSTALL_DIR"
LOG="$STATE_DIR/install-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
echo "Install log: $LOG"
trap 'echo "Installation failed at line $LINENO; log: $LOG" >&2' ERR
mkdir -p "$STATE_DIR/tools/bin"
export PATH="$STATE_DIR/tools/bin:$PATH"
if ((!STEAM_ONLY)); then
    for program in cmake ninja curl unzip c++; do command -v "$program" >/dev/null || die "Missing $program (see --install-deps)"; done
    if ! git lfs version >/dev/null 2>&1; then
        echo "Installing Git LFS 3.7.1 ARM64 locally (SteamOS is not modified)."
        lfs_archive="$STATE_DIR/git-lfs-linux-arm64-v3.7.1.tar.gz"
        curl --fail --location --retry 3 https://github.com/git-lfs/git-lfs/releases/download/v3.7.1/git-lfs-linux-arm64-v3.7.1.tar.gz -o "$lfs_archive"
        printf '%s  %s\n' 73a9c90eeb4312133a63c3eaee0c38c019ea7bfa0953d174809d25b18588dd8d "$lfs_archive" | sha256sum --check
        python3 - "$lfs_archive" "$STATE_DIR/tools/bin/git-lfs" <<'LFS'
import os,pathlib,sys,tarfile
target=pathlib.Path(sys.argv[2])
with tarfile.open(sys.argv[1],'r:gz') as archive:
    matches=[member for member in archive.getmembers() if member.isfile() and pathlib.PurePosixPath(member.name).name=='git-lfs']
    if len(matches)!=1: raise SystemExit('Unexpected Git LFS archive layout')
    with archive.extractfile(matches[0]) as source:
        temp=target.with_suffix('.new'); temp.write_bytes(source.read()); temp.chmod(0o755); os.replace(temp,target)
LFS
        git lfs version
    fi
    cmake --version | head -n 1
    printf '%s\n' '#include <atomic>' '#include <memory>' '#include <charconv>' 'int main(){ std::atomic<std::shared_ptr<int>> p; double d; const char s[]="1.25"; return std::from_chars(s,s+4,d).ec != std::errc{}; }' > "$STATE_DIR/compiler-check.cpp"
    "${CXX:-c++}" -std=c++20 "$STATE_DIR/compiler-check.cpp" -o "$STATE_DIR/compiler-check"
    "$STATE_DIR/compiler-check"
    if [[ ! -e "$INSTALL_DIR/.git" ]]; then
        # .local-install already exists; clone into a temporary child, then move
        # only the new checkout entries. Never delete or overwrite user files.
        [[ $(find "$INSTALL_DIR" -mindepth 1 -maxdepth 1 ! -name .local-install | wc -l) == 0 ]] || die "Install directory contains unrelated files."
        GIT_LFS_SKIP_SMUDGE=1 git clone --branch main --single-branch "$REPO" "$STATE_DIR/checkout"
        shopt -s dotglob
        mv "$STATE_DIR/checkout/"* "$INSTALL_DIR/"
        shopt -u dotglob
        rmdir "$STATE_DIR/checkout"
    fi
    [[ $(git -C "$INSTALL_DIR" rev-parse --show-toplevel) == "$INSTALL_DIR" ]] || die "Not a dedicated checkout"
    [[ $(git -C "$INSTALL_DIR" remote get-url origin) == "$REPO" ]] || die "Unexpected repository origin; refusing to update"
    grep -qxF '/.local-install/' "$INSTALL_DIR/.git/info/exclude" || printf '\n/.local-install/\n' >> "$INSTALL_DIR/.git/info/exclude"
    python3 - "$INSTALL_DIR/game" <<'PY'
import os,pathlib,sys
for proc in pathlib.Path('/proc').iterdir():
    if not proc.name.isdigit(): continue
    try:
        args=(proc/'cmdline').read_bytes().split(b'\0')
        if proc.stat().st_uid==os.getuid() and b'--path' in args and os.fsencode(sys.argv[1]) in args:
            raise SystemExit('Close the running game before updating (PID '+proc.name+').')
    except (OSError,PermissionError): pass
PY
    [[ -z $(git -C "$INSTALL_DIR" status --porcelain --untracked-files=no) ]] || die "Tracked local changes found; preserve/commit them before updating."
    [[ $(git -C "$INSTALL_DIR" branch --show-current) == main ]] || die "Checkout is not on main; refusing to switch your branch."
    git -C "$INSTALL_DIR" fetch origin main
    git -C "$INSTALL_DIR" merge --ff-only origin/main
    [[ $(git -C "$INSTALL_DIR" rev-parse HEAD) == $(git -C "$INSTALL_DIR" rev-parse origin/main) ]] || die "Local main has extra commits; refusing to report it as GitHub main."
    git -C "$INSTALL_DIR" submodule sync --recursive
    git -C "$INSTALL_DIR" submodule update --init --recursive
    git -C "$INSTALL_DIR" lfs install --local
    git -C "$INSTALL_DIR" lfs pull
    git -C "$INSTALL_DIR" lfs fsck
    # Native build, deliberately not the x86-host ARM cross-compilation preset.
    cmake -S "$INSTALL_DIR" -B "$STATE_DIR/build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DRG_BUILD_TESTS=OFF \
        -DRG_BUILD_GODOT_EXTENSION=ON -DRG_GODOT_OUTPUT_NAME=librg_godot.arm64 \
        -DCMAKE_CXX_COMPILER="${CXX:-c++}"
    cmake --build "$STATE_DIR/build" --target rg_godot --parallel "$JOBS"
    # Use the same tested Godot version as the repo's Steam Frame installer.
    GODOT_VERSION=$(python3 - "$INSTALL_DIR/tools/frame_install.ps1" <<'PY'
import pathlib,re,sys
m=re.search(r"\$GodotVersion\s*=\s*'([0-9.]+)'",pathlib.Path(sys.argv[1]).read_text(encoding='utf-8-sig'))
if not m: raise SystemExit('Cannot determine tested Godot version from main')
print(m[1])
PY
)
    GODOT_NAME="Godot_v${GODOT_VERSION}-stable_linux.arm64"
    GODOT="$STATE_DIR/godot/$GODOT_NAME"
    if [[ ! -x "$GODOT" ]]; then
        mkdir -p "$STATE_DIR/godot"
        base="https://github.com/godotengine/godot-builds/releases/download/${GODOT_VERSION}-stable"
        curl --fail --location --retry 3 "$base/$GODOT_NAME.zip" -o "$STATE_DIR/godot/$GODOT_NAME.zip"
        curl --fail --location --retry 3 "$base/SHA512-SUMS.txt" -o "$STATE_DIR/godot/SHA512-SUMS.txt"
        (cd "$STATE_DIR/godot"; grep -E " [*]?${GODOT_NAME//./\.}\.zip$" SHA512-SUMS.txt > selected.sha512; [[ -s selected.sha512 ]]; sha512sum --check selected.sha512; unzip -o "$GODOT_NAME.zip")
        chmod +x "$GODOT"
    fi
    "$GODOT" --headless --path "$INSTALL_DIR/game" --editor --import 2>&1 | tee "$STATE_DIR/import.log"
    if grep -Eq '^(ERROR:|SCRIPT ERROR:)' "$STATE_DIR/import.log"; then die "Godot import reported errors"; fi
    XDG_DATA_HOME="$STATE_DIR/verify/data" XDG_CONFIG_HOME="$STATE_DIR/verify/config" XDG_CACHE_HOME="$STATE_DIR/verify/cache" \
        timeout 180 "$GODOT" --headless --path "$INSTALL_DIR/game" --quit-after 300 -- --bindings-test 2>&1 | tee "$STATE_DIR/bindings.log"
    if grep -Eq '^(ERROR:|SCRIPT ERROR:)' "$STATE_DIR/bindings.log"; then die "Bindings verification reported errors"; fi
    grep -q 'bindings test: ok' "$STATE_DIR/bindings.log" || die "Bindings verification did not complete"
    # JSON config avoids shell interpolation of spaces, quotes or dollar signs.
    python3 - "$STATE_DIR/config.json" "$INSTALL_DIR" "$GODOT" "$MAP_DIR" "$MODE" <<'PY'
import json,pathlib,sys
pathlib.Path(sys.argv[1]).write_text(json.dumps(dict(zip(('root','godot','maps','mode'),sys.argv[2:]))))
PY
    git -C "$INSTALL_DIR" rev-parse HEAD > "$STATE_DIR/installed-commit"
fi
[[ -f "$STATE_DIR/config.json" ]] || die "No completed install; run without --steam-only first."
cat > "$STATE_DIR/launch.py" <<'PY'
#!/usr/bin/env python3
import json,os,pathlib,sys
config=json.loads((pathlib.Path(__file__).parent/'config.json').read_text())
args=sys.argv[1:]; mode=config['mode']
if args and args[0] in ('--vr','--desktop'): mode=args.pop(0)[2:]
if args == ['--flat']: args=['--','--flat']
os.environ.setdefault('RG_G2M_CACHE',config['maps'])
os.environ.setdefault('RG_G2M_HOME',str(pathlib.Path(config['maps'])/'home-r1'))
os.chdir(config['root'])
xr=['--xr-mode','on','--rendering-method','mobile','--rendering-driver','vulkan'] if mode=='vr' else []
os.execv(config['godot'],[config['godot'],'--path',str(pathlib.Path(config['root'])/'game'),*xr,*args])
PY
chmod +x "$STATE_DIR/launch.py"
python3 - "$STATE_DIR" "${BASH_SOURCE[0]}" <<'PY'
import json,os,pathlib,shlex,sys
state=pathlib.Path(sys.argv[1]); config=json.loads((state/'config.json').read_text())
latest=pathlib.Path(config['root'])/'tools/install_linux.sh'
source=latest if latest.exists() else pathlib.Path(sys.argv[2])
temp=state/'installer.sh.new'; temp.write_bytes(source.read_bytes()); os.replace(temp,state/'installer.sh')
cmd=['bash',str(state/'installer.sh'),'--install-dir',config['root'],'--map-dir',config['maps'],'--'+config['mode']]
updater=state/'update.sh'; updater.write_text('#!/usr/bin/env bash\nexec '+shlex.join(cmd)+' "$@"\n'); updater.chmod(0o755)
PY
echo "Launcher: $STATE_DIR/launch.py (or add --desktop / --desktop --flat)"
echo "Updater: bash $STATE_DIR/update.sh"
if [[ ! -d "$MAP_DIR/rhein-main-corridor-r2" ]]; then
    echo "Real-world map store not found in $MAP_DIR. Copy it separately; Flat World works without it."
fi
# Artwork and Steam registration payloads follow. They are embedded so this
# single downloaded script can install an older main before it contains them.
python3 - "$STATE_DIR" <<'ART'
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]); art=root/'artwork'; art.mkdir(exist_ok=True)
# Original vector artwork. Block lettering avoids external fonts/assets.
font={'R':['11110','10001','10001','11110','10100','10010','10001'],
'A':['01110','10001','10001','11111','10001','10001','10001'],
'C':['01111','10000','10000','10000','10000','10000','01111'],
'I':['11111','00100','00100','00100','00100','00100','11111'],
'N':['10001','11001','11001','10101','10011','10011','10001'],
'G':['01111','10000','10000','10111','10001','10001','01111'],
'M':['10001','11011','10101','10101','10001','10001','10001'],
'E':['11111','10000','10000','11110','10000','10000','11111'],
' ':['00000']*7}
def letters(text,x,y,unit):
    rects=[]
    for i,c in enumerate(text):
        for row,line in enumerate(font[c]):
            for col,on in enumerate(line):
                if on=='1': rects.append(f'<rect x="{x+(i*6+col)*unit}" y="{y+row*unit}" width="{unit}" height="{unit}"/>')
    return '<g fill="#f5f7fb">'+''.join(rects)+'</g>'
for name,w,h in [('header',920,430),('capsule',600,900),('hero',1920,620),('icon',256,256)]:
    scale=min(w/920,h/430); cx=w*.52; cy=h*.61
    title='' if name=='icon' else letters('RACING GAME',w*.07,h*.12,min(w/72,h/72))
    car=f'''<g transform="translate({cx-310*scale},{cy-70*scale}) scale({scale})">
      <path d="M0 98 L24 71 L130 52 L206 7 L354 4 L437 55 L552 72 L600 108 L587 145 L16 145 Z" fill="#63b9e3" stroke="#a8ddf3" stroke-width="4"/>
      <path d="M162 54 L221 17 L347 15 L410 57 Z" fill="#112433"/>
      <path d="M22 110 L577 110 L566 132 L37 132 Z" fill="#142532"/>
      <path d="M434 64 L526 69 L550 84 L443 77 Z" fill="#e9faff"/>
      <path d="M7 84 L82 75 L78 88 L6 99 Z" fill="#f68636"/>
      <circle cx="119" cy="137" r="48" fill="#080d14" stroke="#92a5b3" stroke-width="6"/>
      <circle cx="481" cy="137" r="48" fill="#080d14" stroke="#92a5b3" stroke-width="6"/>
      <circle cx="119" cy="137" r="24" fill="#536b7a"/><circle cx="481" cy="137" r="24" fill="#536b7a"/>
      <path d="M79 57 L37 22 L43 14 L133 19 L133 26 L68 28 L105 55" fill="#182938"/>
    </g>'''
    svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">
      <rect width="{w}" height="{h}" fill="#0b121d"/>
      <path d="M{-w*.2} {h} Q{w*.8} {h*.4} {w*1.2} {h*.25} L{w*1.2} {h*.65} Q{w*.5} {h*.5} 0 {h*1.3} Z" fill="#202e40"/>
      <path d="M0 {h*.98} Q{w*.5} {h*.58} {w*1.1} {h*.44}" fill="none" stroke="#f68636" stroke-width="{max(3,w/180)}"/>
      {title}{car}<rect x="0" y="{h-8}" width="{w}" height="8" fill="#f68636"/>
    </svg>'''
    (art/(name+'.svg')).write_text(svg)
project=root/'art_project'; project.mkdir(exist_ok=True)
(project/'project.godot').write_text('config_version=5\n')
(project/'render.gd').write_text('''extends SceneTree
func _initialize():
    var folder := OS.get_cmdline_user_args()[0]
    for name in ["header","capsule","hero","icon"]:
        var image := Image.load_from_file(folder+"/"+name+".svg")
        if image == null or image.save_png(folder+"/"+name+".png") != OK:
            push_error("Artwork render failed: "+name)
            quit(1)
            return
    quit()
''')
ART
GODOT=$(python3 - "$STATE_DIR/config.json" <<'PY'
import json,sys
print(json.load(open(sys.argv[1]))['godot'])
PY
)
"$GODOT" --headless --path "$STATE_DIR/art_project" --script "$STATE_DIR/art_project/render.gd" -- "$STATE_DIR/artwork"
if ((NO_STEAM)); then echo "Installed; Steam registration skipped."; exit 0; fi
cat > "$STATE_DIR/register_steam.py" <<'STEAM'
#!/usr/bin/env python3
"""Preserve binary KeyValues types/unknown fields; no third-party dependencies."""
import json,os,pathlib,re,shutil,struct,sys,tempfile,time,zlib

def read_vdf(data):
    pos=0
    def string():
        nonlocal pos
        end=data.index(b'\0',pos); result=data[pos:end].decode('utf-8'); pos=end+1
        return result
    def obj():
        nonlocal pos
        result={}
        while pos<len(data):
            kind=data[pos]; pos+=1
            if kind==8: return result
            key=string()
            if key in result: raise ValueError('Duplicate VDF key; refusing lossy rewrite')
            if kind==0: value=obj()
            elif kind==1: value=string()
            elif kind in (2,3,4,6,7,10):
                size=8 if kind in (7,10) else 4
                value=data[pos:pos+size]
                if len(value)!=size: raise ValueError('Truncated VDF value')
                pos+=size
            else: raise ValueError(f'Unsupported VDF type {kind}; original file unchanged')
            result[key]=(kind,value)
        raise ValueError('Unterminated VDF object')
    result=obj()
    if pos!=len(data): raise ValueError('Trailing VDF data; original file unchanged')
    return result

def write_vdf(obj):
    result=bytearray()
    for key,(kind,value) in obj.items():
        result+=bytes([kind])+key.encode()+b'\0'
        result+=write_vdf(value) if kind==0 else value.encode()+b'\0' if kind==1 else value
    return bytes(result)+b'\x08'

def atomically_write(path,data):
    fd,temp=tempfile.mkstemp(prefix=path.name+'.',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(temp,path)
    finally:
        if os.path.exists(temp): os.unlink(temp)

def steam_running():
    for proc in pathlib.Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            if proc.stat().st_uid==os.getuid() and (proc/'comm').read_text().strip() in ('steam','steam.exe'):
                return True
        except (OSError,PermissionError): pass
    return False

def register(state,user=''):
    if steam_running():
        raise SystemExit('Game installed. Fully exit Steam, then rerun this installer with --steam-only. Steam files were not changed.')
    home=pathlib.Path.home()
    candidates=[home/'.local/share/Steam',home/'.steam/steam',home/'.steam/root']
    steam=next((p.resolve() for p in candidates if (p/'userdata').is_dir()),None)
    if steam is None: raise SystemExit('Steam userdata not found. Start/login to native Steam once, exit it, then rerun --steam-only.')
    users=[p for p in (steam/'userdata').iterdir() if p.is_dir() and p.name.isdigit() and p.name!='0']
    if user:
        if not user.isdigit(): raise SystemExit('--steam-user must be a numeric userdata account ID')
        users=[p for p in users if p.name==user]
    elif len(users)>1:
        login=steam/'config/loginusers.vdf'
        if login.exists():
            latest=[sid for sid,body in re.findall(r'"(\d{17})"\s*\{([^{}]*)\}',login.read_text()) if re.search(r'"MostRecent"\s*"1"',body,re.I)]
            if len(latest)==1: users=[p for p in users if int(p.name)==int(latest[0])-76561197960265728]
    if len(users)!=1: raise SystemExit('Cannot select Steam account. Rerun --steam-only --steam-user ACCOUNT_ID; candidates: '+', '.join(p.name for p in users))
    config=users[0]/'config'; config.mkdir(exist_ok=True)
    path=config/'shortcuts.vdf'; original=path.read_bytes() if path.exists() else None
    tree=read_vdf(original) if original else {'shortcuts':(0,{})}
    root_key=next((key for key in tree if key.lower()=='shortcuts'),None)
    if root_key is None or tree[root_key][0]!=0: raise ValueError('Invalid shortcuts root')
    entries=tree[root_key][1]; exe='"'+str(state/'launch.py')+'"'; title='Racing Game'
    def field(entry,name):
        return next((v for k,v in entry.items() if k.lower()==name.lower()),None)
    existing=next((entry for kind,entry in entries.values() if kind==0 and field(entry,'Exe')==(1,exe)),None)
    used={struct.unpack('<I',v[1])[0] for kind,e in entries.values() if kind==0 and (v:=field(e,'appid')) and v[0]==2}
    appid=struct.unpack('<I',field(existing,'appid')[1])[0] if existing and field(existing,'appid') else zlib.crc32((exe+title).encode())|0x80000000
    while existing is None and appid in used: appid=((appid+1)&0xffffffff)|0x80000000
    values={'appid':(2,struct.pack('<I',appid)),'AppName':(1,title),'Exe':(1,exe),
        'StartDir':(1,'"'+str(state.parent)+'"'),'icon':(1,str(state/'artwork/icon.png')),
        'LaunchOptions':(1,''),'IsHidden':(2,struct.pack('<I',0)),
        'AllowDesktopConfig':(2,struct.pack('<I',1)),'AllowOverlay':(2,struct.pack('<I',1)),
        'OpenVR':(2,struct.pack('<I',int(json.loads((state/'config.json').read_text())['mode']=='vr'))),
        'tags':(0,{'0':(1,'Racing')})}
    if existing is not None:
        # Preserve user choices such as launch options, tags, AppID and title.
        for key in ('Exe','StartDir','icon','OpenVR'):
            actual=next((k for k in existing if k.lower()==key.lower()),key)
            existing[actual]=values[key]
    else:
        index=0
        while str(index) in entries: index+=1
        entries[str(index)]=(0,values)
    packed=write_vdf(tree)
    if read_vdf(packed)!=tree: raise ValueError('VDF roundtrip failed')
    if original!=packed:
        if original: shutil.copy2(path,path.with_name(path.name+'.racing-game-'+str(time.time_ns())+'.bak'))
        atomically_write(path,packed)
    grid=config/'grid'; grid.mkdir(exist_ok=True)
    for source,suffix in [('header',''),('capsule','p'),('hero','_hero'),('icon','_icon')]:
        shutil.copy2(state/f'artwork/{source}.png',grid/f'{appid}{suffix}.png')
    desktop=home/'.local/share/applications/racing-game.desktop'; desktop.parent.mkdir(parents=True,exist_ok=True)
    def quote(value): return '"'+value.replace('\\','\\\\').replace('"','\\"').replace('$','\\$').replace('`','\\`').replace('%','%%')+'"'
    desktop.write_text('[Desktop Entry]\nType=Application\nName=Racing Game\nExec='+quote(str(state/'launch.py'))+'\nIcon='+str(state/'artwork/icon.png')+'\nTerminal=false\nCategories=Game;\n')
    print(f'Steam shortcut ready: {title}, account {users[0].name}, AppID {appid}; artwork installed. Start Steam to see it.')

if __name__=='__main__': register(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else '')
STEAM
python3 "$STATE_DIR/register_steam.py" "$STATE_DIR" "$STEAM_USER"
echo "Installed main revision: $(cat "$STATE_DIR/installed-commit")"
