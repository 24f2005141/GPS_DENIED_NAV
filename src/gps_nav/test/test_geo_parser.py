import math
import pytest
from gps_nav.geo import ENU, nearby_geodetic, validate
from gps_nav.parser import FlowParser
from gps_nav.demo import arguments, destination, resolve_coordinates


def packet(x, y, quality=255, order='little'):
    motion = x.to_bytes(2, order, signed=True) + y.to_bytes(2, order, signed=True)
    return bytes([0xfe, 4]) + motion + bytes([sum(motion) & 255, quality, 0xaa])


def test_geo_axes_and_wrap():
    origin = ENU(0, 0)
    assert origin.to_local(0, 0) == pytest.approx((0, 0), abs=1e-8)
    assert origin.to_local(0.001, 0)[1] > 110
    assert origin.to_local(0, 0.001)[0] > 110
    assert origin.to_local(-0.001, 0)[1] < -110
    assert origin.to_local(0, -0.001)[0] < -110
    assert abs(ENU(0, 179.999).to_local(0, -179.999)[0] - 222.639) < 1
    assert all(math.isfinite(v) for v in ENU(89.999, 0).to_local(89.999, 0.01))


def test_goal_cli_geographic_coordinates_use_session_origin():
    parser=arguments()
    args=parser.parse_args(['--mode','goal','--origin-lat','37.0',
                            '--origin-lon','-122.0','--goal-lat','37.0',
                            '--goal-lon','-121.99998876551827'])
    goal_geo,goal_local,_=destination(parser,args)
    assert goal_geo==(37.0,-121.99998876551827)
    assert goal_local==pytest.approx((1.0,0.0),abs=1e-5)


def test_prompt_retries_invalid_coordinates_and_zero_distance():
    parser=arguments(); args=parser.parse_args(['--mode','goal'])
    answers=iter(['','nan','91','13.0827','bad','80.2707',
                  '13.0827','80.2707','13.082727','80.2707',''])
    messages=[]
    resolve_coordinates(parser,args,input_fn=lambda _:next(answers),output_fn=messages.append)
    _,goal,_=destination(parser,args)
    assert args.origin_lat==13.0827 and args.origin_lon==80.2707
    assert args.goal_lat==13.082727 and args.goal_lon==80.2707
    assert args.initial_heading_deg==0.0
    assert abs(goal[0]) < 0.01 and 2.9 < goal[1] < 3.1
    assert any('Invalid number' in message for message in messages)
    assert any('within 0.01 m' in message for message in messages)


def test_cli_aliases_and_initial_heading_for_arbitrary_geo_pair():
    parser=arguments()
    args=parser.parse_args(['--mode','goal','--start-lat','13.0827',
                            '--start-lon','80.2707','--destination-lat','13.082727',
                            '--destination-lon','80.2707','--initial-heading-deg','90'])
    resolve_coordinates(parser,args,input_fn=lambda _:pytest.fail('unexpected prompt'))
    _,goal,_=destination(parser,args)
    assert args.initial_heading_deg==90
    assert goal[0]==pytest.approx(0,abs=0.01)
    assert math.hypot(*goal)==pytest.approx(2.987,abs=0.03)


@pytest.mark.parametrize('destination_args', [
    ['--destination-lat','13.0827','--destination-lon','80.2707'],
    ['--destination-lat','91','--destination-lon','80.2707'],
    ['--destination-lat','nan','--destination-lon','80.2707'],
])
def test_invalid_cli_goal_rejected(destination_args):
    parser=arguments()
    args=parser.parse_args(['--mode','goal','--start-lat','13.0827',
                            '--start-lon','80.2707',*destination_args])
    with pytest.raises(SystemExit):
        resolve_coordinates(parser,args,input_fn=lambda _:pytest.fail('unexpected prompt'))


@pytest.mark.parametrize('origin,offset', [((0.0,0.0),(3.0,0.0)),
                                           ((37.0,-122.0),(1.0,1.0)),
                                           ((0.0,179.99999),(3.0,0.0))])
def test_nearby_geodetic_round_trip_for_scripted_goals(origin, offset):
    destination = nearby_geodetic(*origin, *offset)
    assert ENU(*origin).to_local(*destination) == pytest.approx(offset, abs=1e-4)


@pytest.mark.parametrize('lat,lon', [(91, 0), (-91, 0), (0, 181), (0, -181),
                                      (math.nan, 0), (0, math.inf), ('bad', 0), (True, 0)])
def test_invalid_geo(lat, lon):
    with pytest.raises(ValueError):
        validate(lat, lon)


def test_parser_stream_and_recovery():
    p = FlowParser()
    assert p.feed(b'garbage' + packet(-300, 258)[:4]) == []
    found = p.feed(packet(-300, 258)[4:] + packet(0, -1, 0) + packet(1, 2, 255))
    assert [(v.x, v.y, v.quality) for v in found] == [(-300, 258, 255), (0, -1, 0), (1, 2, 255)]
    bad = bytearray(packet(4, 5)); bad[6] ^= 1
    bad_footer = bytearray(packet(4, 5)); bad_footer[8] = 0
    found = p.feed(bytes(bad) + bytes(bad_footer) + packet(7, -8))
    assert [(v.x, v.y) for v in found] == [(7, -8)]
    assert p.checksum_failures == 1 and p.invalid >= 2
    assert FlowParser('big').feed(packet(-100, 200, order='big'))[0].x == -100
