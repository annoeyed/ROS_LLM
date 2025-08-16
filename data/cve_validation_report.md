# ROS CVE 데이터베이스 검증 보고서

##  검증 통계

- **총 CVE 개수**: 61
- **ROS 관련 CVE**: 48
- **거짓 양성**: 13
- **정확도**: 78.7%

##  신뢰도 분포

- **높음 (≥0.7)**: 34개
- **중간 (0.4-0.7)**: 14개
- **낮음 (<0.4)**: 13개

##  ROS 관련 CVE 목록

### CVE-2019-19625
**신뢰도**: 1.00
**ROS 지표**: robot operating system, ros 2, dds
**설명**: SROS 2 0.8.1 (which provides the tools that generate and distribute keys for Robot Operating System 2 and uses the underlying security plugins of DDS from ROS 2) leaks node information due to a leaky ...

### CVE-2019-19627
**신뢰도**: 1.00
**ROS 지표**: robot operating system, ros 2, ros2, dds
**설명**: SROS 2 0.8.1 (after CVE-2019-19625 is mitigated) leaks ROS 2 node-related information regardless of the rtps_protection_kind configuration. (SROS2 provides the tools to generate and distribute keys fo...

### CVE-2022-48217
**신뢰도**: 1.00
**ROS 지표**: robot operating system, tf
**설명**: The tf_remapper_node component 1.1.1 for Robot Operating System (ROS) allows attackers, who control the source code of a different node in the same ROS application, to change a robot's behavior. This ...

### CVE-2024-30961
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the error-t...

### CVE-2024-30962
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Buffer Overflow vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the nav2_amcl pr...

### CVE-2024-30963
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Buffer Overflow vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via a crafted script...

### CVE-2024-30964
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 (ROS2) navigation2- ROS2-humble and navigation 2-humble allows a local attacker to execute arbitrary code via the initial...

### CVE-2024-41644
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via the dyn_param_handler_ component.

### CVE-2024-41645
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2__amcl.

### CVE-2024-41646
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_dwb_controller.

### CVE-2024-41647
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_mppi_controller.

### CVE-2024-41648
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_regulated_pure_pu...

### CVE-2024-41649
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the executor_thread_.

### CVE-2024-41650
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Insecure Permissions vulnerability in Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble allows an attacker to execute arbitrary code via a crafted script to the nav2_costmap_2d.

### CVE-2024-44852
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a segmentation violation via the component theta_star::ThetaStar::isUnsafeToPlan().

### CVE-2024-44853
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component computeControl().

### CVE-2024-44854
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component smoothPlan().

### CVE-2024-44855
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component nav2_navfn_planner().

### CVE-2024-44856
**신뢰도**: 1.00
**ROS 지표**: ros2, open robotics
**설명**: Open Robotics Robotic Operating System 2 ROS2 navigation2 v.humble was discovered to contain a NULL pointer dereference via the component nav2_smac_planner().

### CVE-2023-24010
**신뢰도**: 1.00
**ROS 지표**: ros 2, dds
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnera...

### CVE-2023-24011
**신뢰도**: 1.00
**ROS 지표**: ros 2, dds
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnera...

### CVE-2023-24012
**신뢰도**: 1.00
**ROS 지표**: ros 2, dds
**설명**: An attacker can arbitrarily craft malicious DDS Participants (or ROS 2 Nodes) with valid certificates to compromise and get full control of the attacked secure DDS databus system by exploiting vulnera...

### CVE-2024-41148
**신뢰도**: 1.00
**ROS 지표**: robot operating system, rostopic
**설명**: A code injection vulnerability has been discovered in the Robot Operating System (ROS) 'rostopic' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability lies in ...

### CVE-2024-41921
**신뢰도**: 1.00
**ROS 지표**: robot operating system, rostopic
**설명**: A code injection vulnerability has been discovered in the Robot Operating System (ROS) 'rostopic' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability lies in ...

### CVE-2025-3753
**신뢰도**: 1.00
**ROS 지표**: robot operating system, rosbag
**설명**: A code execution vulnerability has been identified in the Robot Operating System (ROS) 'rosbag' tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability arises from the use of ...

### CVE-2024-41655
**신뢰도**: 0.90
**ROS 지표**: tf2, tf
**설명**: TF2 Item Format helps users format TF2 items to the community standards. Versions of `tf2-item-format` since at least `4.2.6`  and prior to `5.9.14` are vulnerable to a Regular Expression Denial of Se...

### CVE-2020-10271
**신뢰도**: 0.80
**ROS 지표**: robot operating system
**설명**: MiR100, MiR200 and other MiR robots use the Robot Operating System (ROS) default packages exposing the computational graph to all network interfaces, wireless and wired. This is the result of a bad se...

### CVE-2021-37146
**신뢰도**: 0.80
**ROS 지표**: open robotics
**설명**: An infinite loop in Open Robotics ros_comm XMLRPC server in ROS Melodic through 1.4.11 and ROS Noetic through1.15.11 allows remote attackers to cause a Denial of Service in ros_comm via a crafted XMLR...

### CVE-2022-48198
**신뢰도**: 0.80
**ROS 지표**: robot operating system
**설명**: The ntpd_driver component before 1.3.0 and 2.x before 2.2.0 for Robot Operating System (ROS) allows attackers, who control the source code of a different node in the same ROS application, to change a ...

### CVE-2024-37860
**신뢰도**: 0.80
**ROS 지표**: ros2
**설명**: Buffer Overflow vulnerability in Open Robotic Operating System 2 ROS2 navigation2- ROS2-humble&& navigation2-humble allows a local attacker to execute arbitrary code via a crafted .yaml file to the na...

### CVE-2024-37862
**신뢰도**: 0.80
**ROS 지표**: ros2
**설명**: Buffer Overflow vulnerability in Open Robotic Robotic Operating System 2 ROS2 navigation2- ROS2-humble&& navigation2-humble allows a local attacker to execute arbitrary code via a crafted .yaml file t...

### CVE-2024-39780
**신뢰도**: 0.80
**ROS 지표**: robot operating system
**설명**: A YAML deserialization vulnerability was found in the Robot Operating System (ROS) 'dynparam', a command-line tool for getting, setting, and deleting parameters of a dynamically configurable node, aff...

### CVE-2024-39289
**신뢰도**: 0.80
**ROS 지표**: robot operating system
**설명**: A code execution vulnerability has been discovered in the Robot Operating System (ROS) 'rosparam' tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability stems from the use of...

### CVE-2024-39835
**신뢰도**: 0.80
**ROS 지표**: robot operating system
**설명**: A code injection vulnerability has been identified in the Robot Operating System (ROS) 'roslaunch' command-line tool, affecting ROS distributions Noetic Ninjemys and earlier. The vulnerability arises ...

### CVE-2015-7675
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: The "Send as attachment" feature in Ipswitch MOVEit DMZ before 8.2 and MOVEit Mobile before 1.2.2 allow remote authenticated users to bypass authorization and read uploaded files via a valid FileID in...

### CVE-2015-7677
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: The MOVEitISAPI service in Ipswitch MOVEit DMZ before 8.2 provides different error messages depending on whether a FileID exists, which allows remote authenticated users to enumerate FileIDs via the X...

### CVE-2015-7680
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: Ipswitch MOVEit DMZ before 8.2 provides different error messages for authentication attempts depending on whether the user account exists, which allows remote attackers to enumerate usernames via a se...

### CVE-2017-6195
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: Ipswitch MOVEit Transfer (formerly DMZ) allows pre-authentication blind SQL injection. The fixed versions are MOVEit Transfer 2017 9.0.0.201, MOVEit DMZ 8.3.0.30, and MOVEit DMZ 8.2.0.20.

### CVE-2019-14924
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: An issue was discovered in GCDWebServer before 3.5.3. The method moveItem in the GCDWebUploader class checks the FileExtension of newAbsolutePath but not oldAbsolutePath. By leveraging this vulnerabil...

### CVE-2019-18465
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: In Progress MOVEit Transfer 11.1 before 11.1.3, a vulnerability has been found that could allow an attacker to sign in without full credentials via the SSH (SFTP) interface. The vulnerability affects ...

### CVE-2020-8612
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: In Progress MOVEit Transfer 2019.1 before 2019.1.4 and 2019.2 before 2019.2.1, a REST API endpoint failed to adequately sanitize malicious input, which could allow an authenticated attacker to execute...

### CVE-2020-12677
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: An issue was discovered in Progress MOVEit Automation Web Admin. A Web Admin application endpoint failed to adequately sanitize malicious input, which could allow an unauthenticated attacker to execut...

### CVE-2020-28647
**신뢰도**: 0.60
**ROS 지표**: moveit
**설명**: In Progress MOVEit Transfer before 2020.1, a malicious user could craft and store a payload within the application. If a victim within the MOVEit Transfer instance interacts with the stored payload, i...

### CVE-2024-49222
**신뢰도**: 0.60
**ROS 지표**: ament
**설명**: Deserialization of Untrusted Data vulnerability in Amento Tech Pvt ltd WPGuppy allows Object Injection.This issue affects WPGuppy: from n/a through 1.1.0.

### CVE-2024-56280
**신뢰도**: 0.60
**ROS 지표**: ament
**설명**: Incorrect Privilege Assignment vulnerability in Amento Tech Pvt ltd WPGuppy allows Privilege Escalation.This issue affects WPGuppy: from n/a through 1.1.0.

### CVE-2025-24643
**신뢰도**: 0.60
**ROS 지표**: ament
**설명**: Missing Authorization vulnerability in Amento Tech Pvt ltd WPGuppy allows Exploiting Incorrectly Configured Access Control Security Levels. This issue affects WPGuppy: from n/a through 1.1.0.

### CVE-2025-30775
**신뢰도**: 0.60
**ROS 지표**: ament
**설명**: Improper Neutralization of Special Elements used in an SQL Command ('SQL Injection') vulnerability in AmentoTech Private Limited WPGuppy allows SQL Injection. This issue affects WPGuppy: from n/a thro...

### CVE-2025-31920
**신뢰도**: 0.60
**ROS 지표**: ament
**설명**: Improper Neutralization of Special Elements used in an SQL Command ('SQL Injection') vulnerability in AmentoTech WP Guppy allows SQL Injection. This issue affects WP Guppy: from n/a through 4.3.3.

##  거짓 양성 CVE 목록

### CVE-2022-48326
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: Multiple Cross Site Scripting (XSS) vulnerabilities in Mapos 4.39.0 allow attackers to execute arbitrary code. Affects the following parameters: (1) nome, (2) aCliente, (3) eCliente, (4) dCliente, (5)...

### CVE-2023-30394
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: The MoveIt framework 1.1.11 for ROS allows cross-site scripting (XSS) via the API authentication function. NOTE: this issue is disputed by the original reporter because it has "no impact."

### CVE-2015-7678
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: Multiple cross-site request forgery (CSRF) vulnerabilities in Ipswitch MOVEit Mobile 1.2.0.962 and earlier allow remote attackers to hijack the authentication of unspecified victims via unknown vector...

### CVE-2015-7679
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: Cross-site scripting (XSS) vulnerability in Ipswitch MOVEit Mobile before 1.2.2 allows remote attackers to inject arbitrary web script or HTML via the query string to mobile/.

### CVE-2015-7676
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: Ipswitch MOVEit File Transfer (formerly DMZ) 8.1 and earlier, when configured to support file view on download, allows remote authenticated users to conduct cross-site scripting (XSS) attacks by uploa...

### CVE-2018-6545
**신뢰도**: 0.10
**거짓 양성 이유**: 'ross' 포함
**설명**: Ipswitch MoveIt v8.1 is vulnerable to a Stored Cross-Site Scripting (XSS) vulnerability, as demonstrated by human.aspx. Attackers can leverage this vulnerability to send malicious messages to other us...

### CVE-2019-16383
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: MOVEit.DMZ.WebApi.dll in Progress MOVEit Transfer 2018 SP2 before 10.2.4, 2019 before 11.0.2, and 2019.1 before 11.1.1 allows an unauthenticated attacker to gain unauthorized access to the database. D...

### CVE-2019-18464
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In Progress MOVEit Transfer 10.2 before 10.2.6 (2018.3), 11.0 before 11.0.4 (2019.0.4), and 11.1 before 11.1.3 (2019.1.3), multiple SQL Injection vulnerabilities have been found in the REST API that c...

### CVE-2020-8611
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In Progress MOVEit Transfer 2019.1 before 2019.1.4 and 2019.2 before 2019.2.1, multiple SQL Injection vulnerabilities have been found in the REST API that could allow an authenticated attacker to gain...

### CVE-2021-31827
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In Progress MOVEit Transfer before 2021.0 (13.0), a SQL injection vulnerability has been found in the MOVEit Transfer web app that could allow an authenticated attacker to gain unauthorized access to ...

### CVE-2021-33894
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In Progress MOVEit Transfer before 2019.0.6 (11.0.6), 2019.1.x before 2019.1.5 (11.1.5), 2019.2.x before 2019.2.2 (11.2.2), 2020.x before 2020.0.5 (12.0.5), 2020.1.x before 2020.1.4 (12.1.4), and 2021...

### CVE-2021-37614
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In certain Progress MOVEit Transfer versions before 2021.0.3 (aka 13.0.3), SQL injection in the MOVEit Transfer web application could allow an authenticated remote attacker to gain access to the datab...

### CVE-2021-38159
**신뢰도**: 0.10
**거짓 양성 이유**: 'rosoft' 포함
**설명**: In certain Progress MOVEit Transfer versions before 2021.0.4 (aka 13.0.4), SQL injection in the MOVEit Transfer web application could allow an unauthenticated remote attacker to gain access to the dat...

