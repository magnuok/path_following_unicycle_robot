function ans = door_detector(odom,start_coordinates,scan) %input: ranges
%% Odometry in mm
% Start_coordinates in mm, worldcoordinates

% Booleans that execute an event in cause of true
% ans = [detect_door_left, detect_door_right];

odom = [0,0];
start_coordinates = [0,0];
% Process ranges
scan = [1357	1355	1341	1334	1324	1322	1318	1310	1299	1292	1285	1285	1280	1279	1275	1267	1260	1256	1256	1253	1251	1238	1235	1234	1226	1224	1222	1218	1208	1208	1208	1206	1205	1195	1194	1184	1179	1179	1179	1173	1167	1167	1166	1164	1166	1166	1162	1161	1161	1159	1148	1145	1145	1145	1133	1129	1129	1129	1129	1129	1129	1120	1120	1114	1112	1107	1100	1095	1074	1074	1074	1083	1084	1094	1095	1097	1095	1097	1097	1093	1090	1090	1090	1090	1097	1097	1099	1099	1098	1086	1094	1090	1090	1086	1086	1086	1085	1085	1085	1086	1077	1077	1077	1087	1087	1090	1090	1087	1087	1080	1074	1074	1074	1107	1144	1149	1149	1146	1149	1151	1151	1152	1152	1152	1148	1145	1145	1142	1148	1161	1164	1161	1162	1162	1162	1162	1162	1163	1163	1168	1165	1169	1170	1173	1170	1173	1177	1180	1180	1181	1183	1181	1180	1180	1180	1180	1180	1184	1185	1185	1190	1194	1196	1199	1201	1201	1202	1202	1202	1210	1213	1218	1221	1224	1224	1227	1228	1243	1245	1248	1253	1253	1253	1263	1263	1269	1271	1273	1273	1278	1279	1286	1304	1313	1315	1317	1334	1334	1334	1339	1349	1351	1352	1354	1372	1372	1374	1378	1381	1382	1389	1404	1406	1415	1422	1423	1448	1463	1463	1463	1451	1430	1430	1412	1403	1403	1403	1410	1424	1447	1452	1458	1462	1474	1482	1507	1511	1516	1520	1524	1540	1558	1559	1568	1589	1592	1607	1615	1626	1643	1653	1661	1680	1688	1688	1711	1731	1764	1769	1785	1787	1787	1803	1828	1837	1857	1875	1877	1895	1915	1932	1949	1983	1990	2003	2030	2046	2068	2088	2118	2139	2151	2172	2207	2215	2250	2286	2302	2336	2355	2389	2419	2446	2488	2506	2563	2590	2633	2682	2722	2772	2828	2858	2914	2975	3028	3067	3118	3176	3194	3333	0	0	0	7	0	4213	4349	4349	4349	4378	4380	4424	4466	4637	4637	4637	4637	4737	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	6	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	5309	5309	7	0	0	0	0	0	0	0	0	0	0	0	3013	2903	2833	2759	2689	2625	2625	2563	2465	2415	2415	2415	2429	2436	2436	2436	2419	2368	2341	2293	2253	2208	2152	2108	2060	2026	2026	1988	1946	1674	1657	1647	1616	1604	1583	1564	1546	1523	1512	1485	1476	1452	1435	1427	1408	1387	1382	1364	1355	1335	1304	1272	1264	1246	1241	1241	1241	1243	1249	1254	1279	1282	1291	1306	1313	1313	1313	1302	1302	1302	1299	1310	1312	1320	1325	1330	1330	1339	1348	1351	1351	1364	1370	1374	1375	1382	1391	1402	1405	1409	1410	1411	1422	1443	1445	1449	1452	1463	1467	1485	1487	1488	1508	1518	1518	1525	1539	1548	1562	1564	1586	1588	1596	1604	1627	1628	1634	1649	1669	1671	1682	1698	1710	1726	1733	1741	1769	1772	1790	1794	1815	1823	1840	1853	1872	1891	1907	2260	2294	2313	2313	2347	2374	2388	2418	2458	2497	2524	2542	2588	2617	2652	2686	2713	2766	2816	2831	2888	2923	2999	3049	3115	3146	3204	3276	3346	3405	3485	3555	3637	3721	3779	3797	3800	3804	3989	0	7	0	7	0	0	7	0	0	0	0	0	706	706	705	705	705	705	702	703	703	703	703	703	703	696	696	696	702	702	702	702	697	697	695	695	697	695	695	695	699	694	698	691	697	690	697	696	696	696	696	696	695	697	697	697	702	703	703	703	703	702	702	702	702	707	709	709	712	713	713	713	713	713	713	713	718	719	719	719	718	718	718	722	722	723	722	722	725	725	720	725	725	729	732	733	732	736	736	735	735	738	738	746	746	747	752];
scan =[619,618,618,618,614,613,613,611,610,606,601,597,592,590,590,590,596,596,596,596,596,591,591,584,580,580,580,580,578,575,575,573,575,575,579,575,582,582,578,576,576,574,573,573,572,573,573,572,571,571,566,566,571,572,572,572,566,566,566,566,566,566,566,565,564,564,565,565,565,569,569,566,566,566,566,566,566,566,568,567,567,568,567,567,567,574,574,574,573,573,573,568,568,561,555,555,558,559,558,559,559,558,558,561,578,626,635,638,638,638,638,638,638,638,642,642,643,647,647,647,647,647,647,647,652,652,654,654,659,663,659,657,659,658,658,667,669,671,671,671,671,671,671,674,672,677,682,682,680,682,682,689,689,691,696,697,704,705,708,710,712,714,714,714,710,676,676,674,674,662,662,642,642,642,642,647,654,654,655,659,664,675,676,681,681,687,692,696,704,706,708,709,719,721,727,738,739,744,751,756,758,767,775,779,779,780,786,791,802,816,830,833,833,833,833,876,883,906,909,932,934,935,946,950,954,968,971,979,988,996,1000,1002,1009,1019,1036,1038,1048,1048,1070,1079,1106,1120,1123,1124,1124,1123,1120,1119,1115,1113,1104,1098,1094,1094,1094,1127,1134,1149,1173,1186,1203,1216,1227,1242,1266,1282,1306,1318,1336,1363,1376,1410,1438,1468,1494,1516,1554,1579,1607,1644,1677,1712,1750,1792,1831,1875,1917,1960,2000,2074,2106,2160,2211,2274,2338,2407,2469,2578,2653,2855,2951,3026,3026,3123,3132,3848,7,4327,4308,4308,4308,4327,4329,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,0,3939,3747,3668,3605,3543,3489,3423,3382,3324,3265,3212,3159,3113,3054,3009,2973,2923,2890,2855,2814,2777,2739,2714,2714,2714,2722,2728,2728,2728,2732,2692,2692,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,2222,2060,1970,1942,1934,1915,1902,1889,1870,1854,1843,1831,1813,1802,1786,1773,1754,1745,1731,1723,1723,1712,1703,1677,1670,1653,1636,1614,1586,1586,1569,1565,1565,1565,1565,1587,1635,1660,1660,1660,1658,1655,1654,1654,1655,1671,1676,1692,1708,1716,1727,1742,1743,1759,1778,1796,1810,1820,1836,1859,1875,1894,1910,1927,1947,1960,1976,1996,2021,2043,2060,2079,2098,2126,2147,2165,2198,2199,2229,2255,2279,2294,2327,2348,2384,2403,2867,2909,2929,2929,2977,3013,3067,3122,3168,3200,3264,3308,3368,3432,3502,3561,3619,3693,3786,3839,3903,3984,4082,4226,4265,4265,4265,4270,4453,0,7,0,0,0,0,0,0,0,0,0,6,6,6,6,6,1150,1149,1146,1146,1149,1149,1150,1152,1161,1175,1184,1192,1196,1192,1192,1187,1187,1187,1187,1186,1191,1195,1195,1195,1195,1195,1199,1195,1195,1192,1192,1192,1192,1194,1194,1194,1193,1192,1192,1192,1191,1191,1191,1191,1192,1192,1192,1192,1196,1199,1202,1202,1202,1194,1192,1192,1192,1194,1194,1207,1207,1207,1207,1206,1206,1207,1213,1216,1219,1219,1219,1221,1221,1223,1232,1232,1233,1234,1234,1236];
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


% Set rightpoints and leftpoints
leftpoints = []; % [x,y]
rightpoints = [];

% Søker bare for 1 meter frem her.
search_range = 1500;

% deler opp i venstre og høyre liste
for i = 1:length(x)
    if points(i,1) < 0 && points(i,2) < search_range % �nsker ikke � kutte liste mer, for out of range problem i l�kke
        leftpoints = [points(i,:);leftpoints];
    end
    if points(i,1) > 0 && points(i,2) < search_range
        rightpoints = [rightpoints;points(i,:)];
    end
end


% Set thresholds and parameters. door list

% alt i milli. 
door_threshold = 60; % How big norm represents a door? Hvor stor topp på deriviert. Tune opp/ned.
range_threshold = 1500; % How far is odomotry from a existing door? Se bare på dører som er innenfor der odom sier du er
door_distance = 500; % How close should the robot be to the door before is it denoted as detected? distanse fra første dørkarm
detect_door_left = false;
detect_door_right = false;
L_index = 0; % kun brukt for plotting
R_index = 0; % kun brukt for plotting

% [x,y,bol, bol] 3:bol=1=right bol=0=left, 4:bol=1=detected
doors = getdoors();
doors = dlmread('Doors_edit.txt');

% Find close by doors

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

%  Detect right doors
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
            
            R = index;
            
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

%  Detect left doors
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

% PLOTTING. Coment out when running

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
gg = plot(0,0,'g^');
hold on;

gg = plot(leftpoints(:,1),leftpoints(:,2))
hold on
gg = plot(rightpoints(:,1),rightpoints(:,2))
hold on
grid on

if L_index ~=0
    gg = plot(leftpoints(L_index,1),leftpoints(L_index,2),'b*')
    hold on
end
if R_index ~=0
    gg = plot(rightpoints(R_index,1),rightpoints(R_index,2),'r*')
end
hold off

title('Laser scan')
hl=legend('Robot pose' ,'Left wall','Right wall', 'Detected door on left side', 'Detected door on right side',  'AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);


figure(3)
gg = plot(norm_left)
hold on;
gg= plot(L_index, norm_left(L_index),'b*');

title('Norm left wall')
hl=legend('Norms' ,'Detected door on left side','AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);



figure(4)
plot(norm_right)
hold on;
plot(R_index, norm_right(R_index),'r*');

title('Norm right wall')
hl=legend('Norms' ,'Detected door on right side','AutoUpdate','off');
set(hl,'Interpreter','latex')
set(gg,"LineWidth",1.5)
gg=xlabel("x - [m]");
set(gg,"Fontsize",14);
gg=ylabel("y - [m]");
set(gg,"Fontsize",14);


end



