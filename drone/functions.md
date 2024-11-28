# Function list
::Position 규칙::  
GPS 데이터로부터 고도측정 시 오차가 많이 발생하여 lat,lon만을 사용하며, 고도는 AHRS 데이터 사용.

## takeoff( altitude : float )
altitude 고도를 목표로 이동.
> arming 5회 실패시 false
## RTL()
이륙 위치로 복귀 후 착륙 (긴급상황시에도 사용가능.)

## update_mission( mission : position() )
드론 임무 추가
*-* CLP 임무상 이동명령만 필요하기에 position *-*