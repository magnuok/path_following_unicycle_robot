function ans = door_detector(odom,start_coordinates,scan) %input: ranges
% Odometry in mm
% Start_coordinates in mm, worldcoordinates

% Booleans that execute an event in cause of true
% ans = [detect_door_left, detect_door_right];

odom = [20300,13500];
start_coordinates = [0,0];
%% Process ranges
scan = [1357	1355	1341	1334	1324	1322	1318	1310	1299	1292	1285	1285	1280	1279	1275	1267	1260	1256	1256	1253	1251	1238	1235	1234	1226	1224	1222	1218	1208	1208	1208	1206	1205	1195	1194	1184	1179	1179	1179	1173	1167	1167	1166	1164	1166	1166	1162	1161	1161	1159	1148	1145	1145	1145	1133	1129	1129	1129	1129	1129	1129	1120	1120	1114	1112	1107	1100	1095	1074	1074	1074	1083	1084	1094	1095	1097	1095	1097	1097	1093	1090	1090	1090	1090	1097	1097	1099	1099	1098	1086	1094	1090	1090	1086	1086	1086	1085	1085	1085	1086	1077	1077	1077	1087	1087	1090	1090	1087	1087	1080	1074	1074	1074	1107	1144	1149	1149	1146	1149	1151	1151	1152	1152	1152	1148	1145	1145	1142	1148	1161	1164	1161	1162	1162	1162	1162	1162	1163	1163	1168	1165	1169	1170	1173	1170	1173	1177	1180	1180	1181	1183	1181	1180	1180	1180	1180	1180	1184	1185	1185	1190	1194	1196	1199	1201	1201	1202	1202	1202	1210	1213	1218	1221	1224	1224	1227	1228	1243	1245	1248	1253	1253	1253	1263	1263	1269	1271	1273	1273	1278	1279	1286	1304	1313	1315	1317	1334	1334	1334	1339	1349	1351	1352	1354	1372	1372	1374	1378	1381	1382	1389	1404	1406	1415	1422	1423	1448	1463	1463	1463	1451	1430	1430	1412	1403	1403	1403	1410	1424	1447	1452	1458	1462	1474	1482	1507	1511	1516	1520	1524	1540	1558	1559	1568	1589	1592	1607	1615	1626	1643	1653	1661	1680	1688	1688	1711	1731	1764	1769	1785	1787	1787	1803	1828	1837	1857	1875	1877	1895	1915	1932	1949	1983	1990	2003	2030	2046	2068	2088	2118	2139	2151	2172	2207	2215	2250	2286	2302	2336	2355	2389	2419	2446	2488	2506	2563	2590	2633	2682	2722	2772	2828	2858	2914	2975	3028	3067	3118	3176	3194	3333	0	0	0	7	0	4213	4349	4349	4349	4378	4380	4424	4466	4637	4637	4637	4637	4737	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	5309	5309	7	0	0	0	0	0	0	0	0	0	0	0	3013	2903	2833	2759	2689	2625	2625	2563	2465	2415	2415	2415	2429	2436	2436	2436	2419	2368	2341	2293	2253	2208	2152	2108	2060	2026	2026	1988	1946	1674	1657	1647	1616	1604	1583	1564	1546	1523	1512	1485	1476	1452	1435	1427	1408	1387	1382	1364	1355	1335	1304	1272	1264	1246	1241	1241	1241	1243	1249	1254	1279	1282	1291	1306	1313	1313	1313	1302	1302	1302	1299	1310	1312	1320	1325	1330	1330	1339	1348	1351	1351	1364	1370	1374	1375	1382	1391	1402	1405	1409	1410	1411	1422	1443	1445	1449	1452	1463	1467	1485	1487	1488	1508	1518	1518	1525	1539	1548	1562	1564	1586	1588	1596	1604	1627	1628	1634	1649	1669	1671	1682	1698	1710	1726	1733	1741	1769	1772	1790	1794	1815	1823	1840	1853	1872	1891	1907	2260	2294	2313	2313	2347	2374	2388	2418	2458	2497	2524	2542	2588	2617	2652	2686	2713	2766	2816	2831	2888	2923	2999	3049	3115	3146	3204	3276	3346	3405	3485	3555	3637	3721	3779	3797	3800	3804	3989	0	7	0	7	0	0	7	0	0	0	0	0	706	706	705	705	705	705	702	703	703	703	703	703	703	696	696	696	702	702	702	702	697	697	695	695	697	695	695	695	699	694	698	691	697	690	697	696	696	696	696	696	695	697	697	697	702	703	703	703	703	702	702	702	702	707	709	709	712	713	713	713	713	713	713	713	718	719	719	719	718	718	718	722	722	723	722	722	725	725	720	725	725	729	732	733	732	736	736	735	735	738	738	746	746	747	752];
%scan =[850,849,849,845,845,835,832,831,828,825,816,816,816,818,818,819,818,805,802,802,801,802,793,792,793,796,796,796,796,786,782,778,778,778,776,776,776,774,771,770,769,766,766,765,763,760,758,753,753,753,753,753,753,758,758,758,756,756,756,753,751,750,750,748,746,744,744,744,744,748,749,748,744,744,741,740,740,741,741,744,744,742,742,745,742,745,745,744,744,744,743,744,743,743,743,741,741,742,742,745,745,745,745,745,745,750,750,751,751,751,749,749,749,749,749,750,745,748,745,748,748,754,755,758,758,758,759,759,759,759,757,757,757,757,763,763,763,768,772,768,774,771,775,772,775,773,778,779,784,784,784,789,794,794,794,797,797,802,804,805,808,813,814,825,827,827,828,827,828,826,828,831,833,834,843,843,845,845,856,856,857,857,862,863,868,868,873,873,877,879,887,892,894,897,897,913,919,920,920,920,925,927,929,929,946,948,951,953,961,962,973,980,982,987,988,999,1006,1016,1016,1022,1025,1033,1039,1040,1053,1054,1058,1064,1075,1086,1092,1102,1109,1109,1111,1125,1133,1134,1139,1149,1157,1165,1182,1182,1202,1225,1225,1233,1234,1251,1263,1281,1281,1281,1297,1304,1315,1327,1353,1364,1374,1395,1410,1419,1443,1460,1467,1489,1511,1538,1552,1582,1608,1634,1651,1678,1709,1734,1756,1797,1818,1848,1883,1923,1955,1989,2031,2057,2086,2130,2178,2212,2246,2294,2330,2386,2442,2482,2549,2612,2668,2730,2811,2882,2969,3036,3120,3202,3245,3248,0,0,0,0,4443,4441,4440,4441,4443,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3366,3305,3241,3189,3143,3067,3014,2952,2896,2838,2792,2737,2690,2652,2617,2582,2533,2497,2476,2432,2411,2371,2148,2044,2019,2000,1971,1949,1916,1895,1869,1851,1816,1800,1782,1761,1743,1724,1709,1696,1675,1649,1639,1618,1597,1587,1572,1549,1547,1527,1507,1499,1478,1467,1457,1450,1431,1426,1412,1403,1393,1383,1369,1366,1355,1342,1337,1334,1323,1317,1308,1307,1297,1289,1270,1269,1260,1257,1242,1236,1235,1227,1225,1207,1206,1201,1187,1186,1181,1178,1171,1171,1160,1157,1153,1144,1144,1137,1126,1125,1120,1116,1115,1108,1102,1094,1088,1088,1086,1085,1079,1076,1072,1064,1062,1058,1055,1052,1047,1047,1045,1045,1047,1042,1034,1034,1020,1019,1017,1017,1009,1004,1004,1004,993,992,990,986,986,984,983,983,983,981,968,971,968,962,959,958,954,953,953,952,949,951,949,950,949,943,938,940,938,938,932,931,931,929,931,928,927,921,921,921,921,921,918,921,918,917,916,914,912,914,911,911,913,913,913,913,912,905,908,906,908,908,912,912,912,912,910,910,918,927,930,932,951,970,970,969,970,969,966,958,933,922,922,919,911,912,912,911,911,912,911,907,911,907,907,908,911,911,912,912,912,912,912,912,912,913,913,912,920,920,925,925,925,925,933,933,933,933,932,932,930,930,928,934,941,943,946,948,948,949,948,948,949,950,950,950,950,959,959,967,967,969,969,971,971,976,977,984,986,988,988,991,991];
x= [];
y=[];

% Transform data from scan
for n = 1:length(scan)
    % remove points that are less than 15 milli away from scanner
    if scan(n) > 15 
        xn = cosd(-30 + (n-1)*(240/682)) * scan(n);
        yn = sind(-30 + (n-1)*(240/682)) * scan(n);
        x = [x; xn];
        y = [y; yn];
    end

end
points=[x,y];


%% Set rightpoints and leftpoints
leftpoints = []; % [x,y]
rightpoints = [];

% Søker bare for 1 meter frem her.
search_range = 1000;

% deler opp i venstre og høyre liste
for i = 1:length(x)
    if points(i,1) < 0 && points(i,2) < search_range % �nsker ikke � kutte liste mer, for out of range problem i l�kke
        leftpoints = [points(i,:);leftpoints];
    end
    if points(i,1) > 0 && points(i,2) < search_range
        rightpoints = [rightpoints;points(i,:)];
    end
end


%% Set thresholds and parameters. door list

% alt i milli. 
door_threshold = 60; % How big norm represents a door? Hvor stor topp på deriviert. Tune opp/ned.
range_threshold = 1000; % How far is odomotry from a existing door? Se bare på dører som er innenfor der odom sier du er
door_distance = 200; % How close should the robot be to the door before is it denoted as detected? distanse fra første dørkarm
detect_door_left = false;
detect_door_right = false;
L_index = 0; % kun brukt for plotting
R_index = 0; % kun brukt for plotting

% [x,y,bol, bol] 3:bol=1=right bol=0=left, 4:bol=1=detected
doors = getdoors();

%% Find close by doors

% doors close by our odom
nearby_door_right = [];
nearby_door_left = [];
% OBS! Odometry errors will make this a problem after a while... tune threshold

% start_coordinates = robots coordinater i map. Trekker i fra denne på
% dørene for å få dører i robotens frame.

for i = 1:length(doors(:,1))
    range = norm([doors(i,1) - start_coordinates(1),doors(i,2) - start_coordinates(2)] - [odom(1),odom(2)]);
    % if it is close enough and not discovered.
    if range < range_threshold && doors(i,4) == 0 
        if doors(i,3) == 1
            nearby_door_right = [nearby_door_right ; doors(i,:), i]; % adding index because needed to change detected or not parameter to true/false
        else
            nearby_door_left = [nearby_door_left ; doors(i,:), i];
        end
    end
end

%%  Detect right doors
if ~isempty(nearby_door_right)
    
    first_doorframe_pos = [0,0];
    max_norm = 0;
    index = 0;
    % norm between every second point to reduce noise.  
    for n = 1:length(rightpoints(:,1)) - 2 
        % Calculate norm, OBS: benches and elevator could be detected
        norm_right(n) = norm(rightpoints(n,:) - rightpoints(n+2,:));
        % only interested in the first norm that passes threshold. this
        % is the doorframe
        if norm_right(n) > door_threshold
            max_norm = norm_right(n);
            index = n;
            first_doorframe_pos = [rightpoints(index,1),rightpoints(index,2)];
            break;
        end
    end
    
    % If the distance to the door is less than door_distance, we
    % want a DOOR event to occur, and list the door as detected
    if norm(first_doorframe_pos(2)) > 0 && norm(first_doorframe_pos(2)) < door_distance
        R_index = index; % used for plotting
        set_door_detected(nearby_door_right(5));
        detect_door_right = true; % SET GLOBAL RIGHT DOOR TO TRUE
    end
    
end

%%  Detect left doors
if ~isempty(nearby_door_left)
    first_doorframe_pos = [0,0];
    max_norm = 0;
    index = 0;
    
    % norm between every second point to reduce noise.  
    for n = 1:length(leftpoints(:,1)) - 2
        % Calculate norm, OBS: benches and elevator could be detected
        norm_right(n) = norm(leftpoints(n,:) - leftpoints(n+2,:));
        % only interested in the first norm that passes threshold. this
        % is the doorframe
        if norm_right(n) > door_threshold
            max_norm = norm_right(n);
            index = n;
            first_doorframe_pos = [leftpoints(index,1),leftpoints(index,2)];
            break;
        end
    end

    if norm(first_doorframe_pos(2)) > 0 && norm(first_doorframe_pos(2)) < door_distance
        L_index = index;
        set_door_detected(nearby_door_right(5));
        detect_door_left = true; % SET GLOBAL LEFT DOOR TO TRUE
    end
end

% Booleans that execute an event in cause of true
ans = [detect_door_left, detect_door_right];

%% PLOTTING. Coment out when running

for n = 1:length(rightpoints(:,1)) - 2 % norm between every second point to reduce noise.  
    % Find all norms, OBS: benches and elevator could be detected
    norm_right(n) = norm(rightpoints(n,:) - rightpoints(n+2,:));
end

for n = 1:length(leftpoints(:,1)) - 2 % norm between every second point to reduce noise.  
    % Find all norms, OBS: benches and elevator could be detected
    norm_left(n) = norm(leftpoints(n,:) - leftpoints(n+2,:));
end


% Plot rangescan and doors found
figure(1);
plot(leftpoints(:,1),leftpoints(:,2))
hold on
plot(rightpoints(:,1),rightpoints(:,2))
hold on
if L_index ~=0
    plot(leftpoints(L_index,1),leftpoints(L_index,2),'o')
    hold on
end
if R_index ~=0
    plot(rightpoints(R_index,1),rightpoints(R_index,2),'*')
end
hold off
figure(3)
plot(norm_left)
figure(4)
plot(norm_right)
end



