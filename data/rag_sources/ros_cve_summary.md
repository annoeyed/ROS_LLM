# ROS / ROS 2 관련 CVE 목록 (자동 수집)

**마지막 업데이트**: 2025-08-13T10:33:16.711601
**총 CVE 개수**: 61

## 통계

- Critical: 16개
- High: 29개
- Medium: 7개
- Low: 0개

## 키워드별 분류

- ros: 32개
- dds: 5개
- sros: 5개
- ament: 14개
- tf: 2개
- rcl: 1개
- moveit: 21개
- navigation: 18개
- rostopic: 2개
- rosbag: 1개

## CVE 상세 목록

### CVE-2019-19625
**심각도**: MEDIUM
**CVSS 점수**: 5.3
**설명**: SROS 2 0.8.1 (which provides the tools that generate and distribute keys for Robot Operating System 2 and uses the underlying security plugins of DDS from ROS 2) leaks node information due to a leaky default configuration as indicated in the policy/defaults/dds/governance.xml document.
**키워드**: ros, dds, sros
**발행일**: 2019-12-06T16:15:11.247

---

### CVE-2019-19627
**심각도**: MEDIUM
**CVSS 점수**: 5.3
**설명**: SROS 2 0.8.1 (after CVE-2019-19625 is mitigated) leaks ROS 2 node-related information regardless of the rtps_protection_kind configuration. (SROS2 provides the tools to generate and distribute keys for Robot Operating System 2 and uses the underlying security plugins of DDS from ROS 2.)
**키워드**: ros, dds, sros
**발행일**: 2019-12-06T16:15:11.357

---

### CVE-2020-10271
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: MiR100, MiR200 and other MiR robots use the Robot Operating System (ROS) default packages exposing the computational graph to all network interfaces, wireless and wired. This is the result of a bad set up and can be mitigated by appropriately configuring ROS and/or applying custom patches as appropriate. Currently, the ROS computational graph can be accessed fully from the wired exposed ports. In combination with other flaws such as CVE-2020-10269, the computation graph can also be fetched and interacted from wireless networks. This allows a malicious operator to take control of the ROS logic and correspondingly, the complete robot given that MiR's operations are centered around the framework (ROS).
**키워드**: ros, ament
**발행일**: 2020-06-24T05:15:12.847

---

### CVE-2021-37146
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: An infinite loop in Open Robotics ros_comm XMLRPC server in ROS Melodic through 1.4.11 and ROS Noetic through1.15.11 allows remote attackers to cause a Denial of Service in ros_comm via a crafted XMLRPC call.
**발행일**: 2021-09-28T13:15:07.253

---

### CVE-2022-48198
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: The ntpd_driver component before 1.3.0 and 2.x before 2.2.0 for Robot Operating System (ROS) allows attackers, who control the source code of a different node in the same ROS application, to change a robot's behavior. This occurs because a topic name depends on the attacker-controlled time_ref_topic parameter.
**키워드**: ros
**발행일**: 2023-01-01T07:15:10.187

---

### CVE-2022-48217
**심각도**: HIGH
**CVSS 점수**: 8.1
**설명**: The tf_remapper_node component 1.1.1 for Robot Operating System (ROS) allows attackers, who control the source code of a different node in the same ROS application, to change a robot's behavior. This occurs because a topic name depends on the attacker-controlled old_tf_topic_name and/or new_tf_topic_name parameter. NOTE: the vendor's position is "it is the responsibility of the programmer to make sure that only known and required parameters are set and unexpected parameters are not."
**키워드**: ros, tf
**발행일**: 2023-01-04T19:15:09.517

---

### CVE-2022-48326
**심각도**: MEDIUM
**CVSS 점수**: 6.1
**설명**: Multiple Cross Site Scripting (XSS) vulnerabilities in Mapos 4.39.0 allow attackers to execute arbitrary code. Affects the following parameters: (1) nome, (2) aCliente, (3) eCliente, (4) dCliente, (5) vCliente, (6) aProduto, (7) eProduto, (8) dProduto, (9) vProduto, (10) aServico, (11) eServico, (12) dServico, (13) vServico, (14) aOs, (15) eOs, (16) dOs, (17) vOs, (18) aVenda, (19) eVenda, (20) dVenda, (21) vVenda, (22) aGarantia, (23) eGarantia, (24) dGarantia, (25) vGarantia, (26) aArquivo, (27) eArquivo, (28) dArquivo, (29) vArquivo, (30) aPagamento, (31) ePagamento, (32) dPagamento, (33) vPagamento, (34) aLancamento, (35) eLancamento, (36) dLancamento, (37) vLancamento, (38) cUsuario, (39) cEmitente, (40) cPermissao, (41) cBackup, (42) cAuditoria, (43) cEmail, (44) cSistema, (45) rCliente, (46) rProduto, (47) rServico, (48) rOs, (49) rVenda, (50) rFinanceiro, (51) aCobranca, (52) eCobranca, (53) dCobranca, (54) vCobranca, (55) situacao, (56) idPermissao, (57) id in file application/controllers/Permissoes.php; (58) precoCompra, (59) precoVenda, (60) descricao, (61) unidade, (62) estoque, (63) estoqueMinimo, (64) idProdutos, (65) id, (66) estoqueAtual in file application/controllers/Produtos.php.
**키워드**: ament, rcl
**발행일**: 2023-02-16T21:15:14.133

---

### CVE-2023-30394
**심각도**: MEDIUM
**CVSS 점수**: 6.1
**설명**: The MoveIt framework 1.1.11 for ROS allows cross-site scripting (XSS) via the API authentication function. NOTE: this issue is disputed by the original reporter because it has "no impact."
**키워드**: moveit
**발행일**: 2023-05-11T19:15:09.497

---

### CVE-2024-30961
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the error-thrown mechanism in nav2_bt_navigator.
**키워드**: ros, navigation
**발행일**: 2024-12-05T23:15:05.030

---

### CVE-2024-30962
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: Buffer Overflow vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the nav2_amcl process
**키워드**: ros, navigation
**발행일**: 2024-12-05T23:15:05.147

---

### CVE-2024-30963
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: Buffer Overflow vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via a crafted script.
**키워드**: ros, navigation
**발행일**: 2024-12-05T23:15:05.267

---

### CVE-2024-30964
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the initial_pose_sub thread created by nav2_bt_navigator
**키워드**: ros, navigation
**발행일**: 2024-12-05T23:15:05.390

---

### CVE-2024-37860
**심각도**: HIGH
**CVSS 점수**: 7.3
**설명**: Buffer Overflow vulnerability in Open Robotic Operating System 2 ROS2 navigation2- ROS2-humble&& navigation2-humble allows a local attacker to execute arbitrary code via a crafted .yaml file to the nav2_amcl process
**키워드**: ros, navigation
**발행일**: 2024-12-05T23:15:05.510

---

### CVE-2024-37862
**심각도**: HIGH
**CVSS 점수**: 7.3
**설명**: Buffer Overflow vulnerability in Open Robotic Robotic Operating System 2 ROS2 navigation2- ROS2-humble&& navigation2-humble allows a local attacker to execute arbitrary code via a crafted .yaml file to the nav2_planner process.
**키워드**: ros, ament, navigation
**발행일**: 2024-12-05T23:15:05.767

---

### CVE-2024-41644
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via the dyn_param_handler_ component.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:20.450

---

### CVE-2024-41645
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2__amcl.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:20.563

---

### CVE-2024-41646
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_dwb_controller.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:20.683

---

### CVE-2024-41647
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_mppi_controller.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:20.807

---

### CVE-2024-41648
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_regulated_pure_pursuit_controller.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:20.920

---

### CVE-2024-41649
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the executor_thread_.
**키워드**: ros, ament, navigation
**발행일**: 2024-12-06T22:15:21.037

---

### CVE-2024-41650
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_costmap_2d.
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.163

---

### CVE-2024-44852
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a segmentation violation via the component theta_star::ThetaStar::isUnsafeToPlan().
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.277

---

### CVE-2024-44853
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component computeControl().
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.390

---

### CVE-2024-44854
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component smoothPlan().
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.500

---

### CVE-2024-44855
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component nav2_navfn_planner().
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.630

---

### CVE-2024-44856
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component nav2_smac_planner().
**키워드**: ros, navigation
**발행일**: 2024-12-06T22:15:21.753

---

### CVE-2023-24010
**심각도**: HIGH
**CVSS 점수**: 8.2
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnerable attributes in the configuration of PKCS#7 certificate’s validation. This is caused by a non-compliant implementation of permission document verification used by some DDS vendors. Specifically, an improper use of the OpenSSL PKCS7_verify function used to validate S/MIME signatures.
**키워드**: ros, dds, sros
**발행일**: 2025-01-09T15:15:11.467

---

### CVE-2023-24011
**심각도**: HIGH
**CVSS 점수**: 8.2
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnerable attributes in the configuration of PKCS#7 certificate’s validation. This is caused by a non-compliant implementation of permission document verification used by some DDS vendors. Specifically, an improper use of the OpenSSL PKCS7_verify function used to validate S/MIME signatures.
**키워드**: ros, dds, sros
**발행일**: 2025-01-09T15:15:11.657

---

### CVE-2023-24012
**심각도**: HIGH
**CVSS 점수**: 8.2
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnerable attributes in the configuration of PKCS#7 certificate’s validation. This is caused by a non-compliant implementation of permission document verification used by some DDS vendors. Specifically, an improper use of the OpenSSL PKCS7_verify function used to validate S/MIME signatures.
**키워드**: ros, dds, sros
**발행일**: 2025-01-09T15:15:11.810

---

### CVE-2024-39780
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A YAML deserialization vulnerability was found in the Robot Operating System (ROS) 'dynparam', a command-line tool for getting, setting, and deleting parameters of a dynamically configurable node, affecting ROS distributions Noetic and earlier. The issue is caused by the use of the yaml.load() function in the 'set' and 'get' verbs, and allows for the creation of arbitrary Python objects. Through this flaw, a local or remote user can craft and execute arbitrary Python code.
**키워드**: ros, ament
**발행일**: 2025-04-02T08:15:13.720

---

### CVE-2024-39289
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A code execution vulnerability has been discovered in the Robot Operating System (ROS) 'rosparam' tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability stems from the use of the eval() function to process unsanitized, user-supplied parameter values via special converters for angle representations in radians. This flaw allowed attackers to craft and execute arbitrary Python code.
**키워드**: ros
**발행일**: 2025-07-17T20:15:27.230

---

### CVE-2024-39835
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A code injection vulnerability has been identified in the Robot Operating System (ROS) 'roslaunch' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability arises from the use of the eval() method to process user-supplied, unsanitized parameter values within the substitution args mechanism, which roslaunch evaluates before launching a node. This flaw allows attackers to craft and execute arbitrary Python code.
**키워드**: ros
**발행일**: 2025-07-17T20:15:27.400

---

### CVE-2024-41148
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A code injection vulnerability has been discovered in the Robot Operating System (ROS) 'rostopic' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability lies in the 'hz' verb, which reports the publishing rate of a topic and accepts a user-provided Python expression via the --filter option. This input is passed directly to the eval() function without sanitization, allowing a local user to craft and execute arbitrary code.
**키워드**: ros, ament, rostopic
**발행일**: 2025-07-17T20:15:27.570

---

### CVE-2024-41921
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A code injection vulnerability has been discovered in the Robot Operating System (ROS) 'rostopic' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability lies in the 'echo' verb, which allows a user to introspect a ROS topic and accepts a user-provided Python expression via the --filter option. This input is passed directly to the eval() function without sanitization, allowing a local user to craft and execute arbitrary code.
**키워드**: ros, ament, rostopic
**발행일**: 2025-07-17T20:15:27.750

---

### CVE-2025-3753
**심각도**: HIGH
**CVSS 점수**: 7.8
**설명**: A code execution vulnerability has been identified in the Robot Operating System (ROS) 'rosbag' tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability arises from the use of the eval() function to process unsanitized, user-supplied input in the 'rosbag filter' command. This flaw enables attackers to craft and execute arbitrary Python code.
**키워드**: ros, rosbag
**발행일**: 2025-07-17T20:15:29.683

---

### CVE-2015-7675
**심각도**: Unknown
**설명**: The "Send as attachment" feature in Ipswitch MOVEit DMZ before 8.2 and MOVEit Mobile before 1.2.2 allow remote authenticated users to bypass authorization and read uploaded files via a valid FileID in the (1) serverFileIds parameter to mobile/sendMsg or (2) arg01 parameter to human.aspx.
**키워드**: moveit
**발행일**: 2016-02-10T15:59:00.100

---

### CVE-2015-7677
**심각도**: Unknown
**설명**: The MOVEitISAPI service in Ipswitch MOVEit DMZ before 8.2 provides different error messages depending on whether a FileID exists, which allows remote authenticated users to enumerate FileIDs via the X-siLock-FileID parameter in a download action to MOVEitISAPI/MOVEitISAPI.dll.
**키워드**: moveit
**발행일**: 2016-02-10T15:59:01.350

---

### CVE-2015-7678
**심각도**: Unknown
**설명**: Multiple cross-site request forgery (CSRF) vulnerabilities in Ipswitch MOVEit Mobile 1.2.0.962 and earlier allow remote attackers to hijack the authentication of unspecified victims via unknown vectors.
**키워드**: moveit
**발행일**: 2016-02-10T15:59:02.490

---

### CVE-2015-7679
**심각도**: Unknown
**설명**: Cross-site scripting (XSS) vulnerability in Ipswitch MOVEit Mobile before 1.2.2 allows remote attackers to inject arbitrary web script or HTML via the query string to mobile/.
**키워드**: moveit
**발행일**: 2016-02-10T15:59:03.413

---

### CVE-2015-7680
**심각도**: Unknown
**설명**: Ipswitch MOVEit DMZ before 8.2 provides different error messages for authentication attempts depending on whether the user account exists, which allows remote attackers to enumerate usernames via a series of SOAP requests to machine.aspx.
**키워드**: moveit
**발행일**: 2016-02-10T15:59:04.507

---

### CVE-2015-7676
**심각도**: Unknown
**설명**: Ipswitch MOVEit File Transfer (formerly DMZ) 8.1 and earlier, when configured to support file view on download, allows remote authenticated users to conduct cross-site scripting (XSS) attacks by uploading HTML files.
**키워드**: moveit
**발행일**: 2016-04-15T15:59:01.080

---

### CVE-2017-6195
**심각도**: Unknown
**설명**: Ipswitch MOVEit Transfer (formerly DMZ) allows pre-authentication blind SQL injection. The fixed versions are MOVEit Transfer 2017 9.0.0.201, MOVEit DMZ 8.3.0.30, and MOVEit DMZ 8.2.0.20.
**키워드**: moveit
**발행일**: 2017-05-18T06:29:00.217

---

### CVE-2018-6545
**심각도**: Unknown
**설명**: Ipswitch MoveIt v8.1 is vulnerable to a Stored Cross-Site Scripting (XSS) vulnerability, as demonstrated by human.aspx. Attackers can leverage this vulnerability to send malicious messages to other users in order to steal session cookies and launch client-side attacks.
**키워드**: moveit
**발행일**: 2018-02-02T09:29:00.803

---

### CVE-2019-14924
**심각도**: Unknown
**설명**: An issue was discovered in GCDWebServer before 3.5.3. The method moveItem in the GCDWebUploader class checks the FileExtension of newAbsolutePath but not oldAbsolutePath. By leveraging this vulnerability, an adversary can make an inaccessible file be available (the credential of the app, for instance).
**키워드**: moveit
**발행일**: 2019-08-10T19:15:10.920

---

### CVE-2019-16383
**심각도**: CRITICAL
**CVSS 점수**: 9.4
**설명**: MOVEit.DMZ.WebApi.dll in Progress MOVEit Transfer 2018 SP2 before 10.2.4, 2019 before 11.0.2, and 2019.1 before 11.1.1 allows an unauthenticated attacker to gain unauthorized access to the database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database, or may be able to alter the database via the REST API, aka SQL Injection.
**키워드**: moveit
**발행일**: 2019-09-24T15:15:15.050

---

### CVE-2019-18464
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: In Progress MOVEit Transfer 10.2 before 10.2.6 (2018.3), 11.0 before 11.0.4 (2019.0.4), and 11.1 before 11.1.3 (2019.1.3), multiple SQL Injection vulnerabilities have been found in the REST API that could allow an unauthenticated attacker to gain unauthorized access to the database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database or may be able to alter the database.
**키워드**: moveit
**발행일**: 2019-10-31T17:15:10.400

---

### CVE-2019-18465
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: In Progress MOVEit Transfer 11.1 before 11.1.3, a vulnerability has been found that could allow an attacker to sign in without full credentials via the SSH (SFTP) interface. The vulnerability affects only certain SSH (SFTP) configurations, and is applicable only if the MySQL database is being used.
**키워드**: moveit
**발행일**: 2019-10-31T17:15:10.477

---

### CVE-2020-8611
**심각도**: HIGH
**CVSS 점수**: 8.8
**설명**: In Progress MOVEit Transfer 2019.1 before 2019.1.4 and 2019.2 before 2019.2.1, multiple SQL Injection vulnerabilities have been found in the REST API that could allow an authenticated attacker to gain unauthorized access to MOVEit Transfer's database via the REST API. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database in addition to executing SQL statements that alter or destroy database elements.
**키워드**: moveit
**발행일**: 2020-02-14T18:15:09.963

---

### CVE-2020-8612
**심각도**: CRITICAL
**CVSS 점수**: 9.0
**설명**: In Progress MOVEit Transfer 2019.1 before 2019.1.4 and 2019.2 before 2019.2.1, a REST API endpoint failed to adequately sanitize malicious input, which could allow an authenticated attacker to execute arbitrary code in a victim's browser, aka XSS.
**키워드**: ament, moveit
**발행일**: 2020-02-14T19:15:10.590

---

### CVE-2020-12677
**심각도**: MEDIUM
**CVSS 점수**: 6.1
**설명**: An issue was discovered in Progress MOVEit Automation Web Admin. A Web Admin application endpoint failed to adequately sanitize malicious input, which could allow an unauthenticated attacker to execute arbitrary code in a victim's browser, aka XSS. This affects 2018 - 2018.0 prior to 2018.0.3, 2018 SP1 - 2018.2 prior to 2018.2.3, 2018 SP2 - 2018.3 prior to 2018.3.7, 2019 - 2019.0 prior to 2019.0.3, 2019.1 - 2019.1 prior to 2019.1.2, and 2019.2 - 2019.2 prior to 2019.2.2.
**키워드**: ament, moveit
**발행일**: 2020-05-14T18:15:12.173

---

### CVE-2020-28647
**심각도**: MEDIUM
**CVSS 점수**: 5.4
**설명**: In Progress MOVEit Transfer before 2020.1, a malicious user could craft and store a payload within the application. If a victim within the MOVEit Transfer instance interacts with the stored payload, it could invoke and execute arbitrary code within the context of the victim's browser (XSS).
**키워드**: moveit
**발행일**: 2020-11-17T14:15:11.417

---

### CVE-2021-31827
**심각도**: HIGH
**CVSS 점수**: 8.8
**설명**: In Progress MOVEit Transfer before 2021.0 (13.0), a SQL injection vulnerability has been found in the MOVEit Transfer web app that could allow an authenticated attacker to gain unauthorized access to MOVEit Transfer's database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database in addition to executing SQL statements that alter or destroy database elements. This is in MOVEit.DMZ.WebApp in SILHuman.vb.
**키워드**: moveit
**발행일**: 2021-05-18T12:15:07.773

---

### CVE-2021-33894
**심각도**: HIGH
**CVSS 점수**: 8.8
**설명**: In Progress MOVEit Transfer before 2019.0.6 (11.0.6), 2019.1.x before 2019.1.5 (11.1.5), 2019.2.x before 2019.2.2 (11.2.2), 2020.x before 2020.0.5 (12.0.5), 2020.1.x before 2020.1.4 (12.1.4), and 2021.x before 2021.0.1 (13.0.1), a SQL injection vulnerability exists in SILUtility.vb in MOVEit.DMZ.WebApp in the MOVEit Transfer web app. This could allow an authenticated attacker to gain unauthorized access to the database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database and/or execute SQL statements that alter or delete database elements.
**키워드**: moveit
**발행일**: 2021-06-09T19:15:09.683

---

### CVE-2021-37614
**심각도**: HIGH
**CVSS 점수**: 8.8
**설명**: In certain Progress MOVEit Transfer versions before 2021.0.3 (aka 13.0.3), SQL injection in the MOVEit Transfer web application could allow an authenticated remote attacker to gain access to the database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database, or execute SQL statements that alter or delete database elements, via crafted strings sent to unique MOVEit Transfer transaction types. The fixed versions are 2019.0.7 (11.0.7), 2019.1.6 (11.1.6), 2019.2.3 (11.2.3), 2020.0.6 (12.0.6), 2020.1.5 (12.1.5), and 2021.0.3 (13.0.3).
**키워드**: moveit
**발행일**: 2021-08-05T20:15:09.497

---

### CVE-2021-38159
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: In certain Progress MOVEit Transfer versions before 2021.0.4 (aka 13.0.4), SQL injection in the MOVEit Transfer web application could allow an unauthenticated remote attacker to gain access to the database. Depending on the database engine being used (MySQL, Microsoft SQL Server, or Azure SQL), an attacker may be able to infer information about the structure and contents of the database, or execute SQL statements that alter or delete database elements, via crafted strings sent to unique MOVEit Transfer transaction types. The fixed versions are 2019.0.8 (11.0.8), 2019.1.7 (11.1.7), 2019.2.4 (11.2.4), 2020.0.7 (12.0.7), 2020.1.6 (12.1.6), and 2021.0.4 (13.0.4).
**키워드**: moveit
**발행일**: 2021-08-07T17:15:07.117

---

### CVE-2024-49222
**심각도**: CRITICAL
**CVSS 점수**: 9.8
**설명**: Deserialization of Untrusted Data vulnerability in Amento Tech Pvt ltd WPGuppy allows Object Injection.This issue affects WPGuppy: from n/a through 1.1.0.
**키워드**: ament
**발행일**: 2025-01-07T11:15:07.080

---

### CVE-2024-56280
**심각도**: HIGH
**CVSS 점수**: 8.8
**설명**: Incorrect Privilege Assignment vulnerability in Amento Tech Pvt ltd WPGuppy allows Privilege Escalation.This issue affects WPGuppy: from n/a through 1.1.0.
**키워드**: ament
**발행일**: 2025-01-07T11:15:09.820

---

### CVE-2025-24643
**심각도**: MEDIUM
**CVSS 점수**: 6.5
**설명**: Missing Authorization vulnerability in Amento Tech Pvt ltd WPGuppy allows Exploiting Incorrectly Configured Access Control Security Levels. This issue affects WPGuppy: from n/a through 1.1.0.
**키워드**: ament
**발행일**: 2025-02-03T15:15:28.537

---

### CVE-2025-30775
**심각도**: HIGH
**CVSS 점수**: 8.5
**설명**: Improper Neutralization of Special Elements used in an SQL Command ('SQL Injection') vulnerability in AmentoTech Private Limited WPGuppy allows SQL Injection. This issue affects WPGuppy: from n/a through 1.1.3.
**키워드**: ament
**발행일**: 2025-03-27T11:15:38.673

---

### CVE-2025-31920
**심각도**: HIGH
**CVSS 점수**: 8.5
**설명**: Improper Neutralization of Special Elements used in an SQL Command ('SQL Injection') vulnerability in AmentoTech WP Guppy allows SQL Injection. This issue affects WP Guppy: from n/a through 4.3.3.
**키워드**: ament
**발행일**: 2025-06-09T16:15:39.187

---

### CVE-2024-41655
**심각도**: HIGH
**CVSS 점수**: 7.5
**설명**: TF2 Item Format helps users format TF2 items to the community standards. Versions of `tf2-item-format` since at least `4.2.6`  and prior to `5.9.14` are vulnerable to a Regular Expression Denial of Service (ReDoS) attack when parsing crafted user input. This vulnerability can be exploited by an attacker to perform DoS attacks on any service that uses any `tf2-item-format` to parse user input. Version `5.9.14` contains a fix for the issue.
**키워드**: tf
**발행일**: 2024-07-23T15:15:05.207

---

