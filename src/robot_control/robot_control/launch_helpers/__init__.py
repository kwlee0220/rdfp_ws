"""launch 파일들이 공유하는 helper 모듈 모음.

예전에는 `launch/` 디렉터리의 sibling 파일이었고 각 launch 가
`sys.path.insert(0, os.path.dirname(__file__))` 로 끌어다 썼다. 그 방식은 한
디렉터리 안에서만 동작해서, launch 계열이 패키지로 갈라지면 상위 패키지가
하위 helper 를 가져올 수 없다. 설치되는 정규 모듈로 승격해 해결한다.
"""
