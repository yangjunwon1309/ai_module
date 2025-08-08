"""
reltr_scene_graph 패키지 초기화.
RelTR 디렉터리를 최상위 모듈로 노출시켜
    import RelTR ...
구문이 그대로 동작하도록 경로를 주입한다.
"""
from pathlib import Path
import importlib.util, sys

_this_dir = Path(__file__).resolve().parent
_reltr_dir = _this_dir / "RelTR"

# 1. PYTHONPATH에 reltr_scene_graph 경로 넣기 (없으면)
if str(_this_dir) not in sys.path:
    sys.path.insert(0, str(_this_dir))

# 2. 'RelTR' 최상위 모듈로 등록
if "RelTR" not in sys.modules:
    init_py = _reltr_dir / "__init__.py"
    # 없으면 빈 모듈이라도 만들기
    if not init_py.exists():
        init_py.touch()
    spec = importlib.util.spec_from_file_location("RelTR", init_py)
    reltr_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reltr_mod)
    sys.modules["RelTR"] = reltr_mod
