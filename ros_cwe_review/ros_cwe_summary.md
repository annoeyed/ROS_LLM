# ROS / ROS 2 관련 CWE 목록 (자동 수집)

**마지막 업데이트**: 2025-08-16T11:44:05.195843
**총 CWE 개수**: 176

## 통계

### 카테고리별 분류
- input_validation: 2개
- file_operations: 2개

### ROS 컴포넌트별 분류

## CWE 상세 목록

### 1051 - Initialization with Hard-Coded Network Resource Configuration Data
**설명**: The product initializes data using hard-coded values that act as network resource identifiers.
**상태**: Incomplete

---

### 1066 - Missing Serialization Control Element
**설명**: The product contains a serializable data element that does not
					have an associated serialization method.
**상태**: Incomplete

---

### 1080 - Source Code File with Excessive Number of Lines of Code
**설명**: A source code file has too many lines of
					code.
**상태**: Incomplete

---

### 1084 - Invokable Control Element with Excessive File or Data Access Operations
**설명**: A function or method contains too many
					operations that utilize a data manager or file resource.
**상태**: Incomplete

---

### 1118 - Insufficient Documentation of Error Handling Techniques
**설명**: The documentation does not sufficiently describe the techniques
					that are used for error handling, exception processing, or similar
					mechanisms.
**상태**: Incomplete

---

### 119 - Improper Restriction of Operations within the Bounds of a Memory Buffer
**설명**: The product performs operations on a memory buffer, but it reads from or writes to a memory location outside the buffer's intended boundary. This may result in read or write operations on unexpected memory locations that could be linked to other variables, data structures, or internal program data.
**상태**: Stable

---

### 120 - Buffer Copy without Checking Size of Input ('Classic Buffer Overflow')
**설명**: The product copies an input buffer to an output buffer without verifying that the size of the input buffer is less than the size of the output buffer, leading to a buffer overflow.

A buffer overflow condition exists when a product attempts to put more data in a buffer than it can hold, or when it attempts to put data in a memory area outside of the boundaries of a buffer. The simplest type of error, and the most common cause of buffer overflows, is the "classic" case in which the product copies the buffer without restricting how much is copied. Other variants exist, but the existence of a classic overflow strongly suggests that the programmer is not considering even the most basic of security protections.
**상태**: Incomplete

---

### 121 - Stack-based Buffer Overflow
**설명**: A stack-based buffer overflow condition is a condition where the buffer being overwritten is allocated on the stack (i.e., is a local variable or, rarely, a parameter to a function).
**상태**: Draft

---

### 122 - Heap-based Buffer Overflow
**설명**: A heap overflow condition is a buffer overflow, where the buffer that can be overwritten is allocated in the heap portion of memory, generally meaning that the buffer was allocated using a routine such as malloc().
**상태**: Draft

---

### 1223 - Race Condition for Write-Once Attributes
**설명**: A write-once register in hardware design is programmable by an untrusted software component earlier than the trusted software component, resulting in a race condition issue.


                
**상태**: Incomplete

---

### 1236 - Improper Neutralization of Formula Elements in a CSV File
**설명**: The product saves user-provided information into a Comma-Separated Value (CSV) file, but it does not neutralize or incorrectly neutralizes special elements that could be interpreted as a command when the file is opened by a spreadsheet product.

User-provided data is often saved to traditional databases.  This data can be exported to a CSV file, which allows users to read the data using spreadsheet software such as Excel, Numbers, or Calc.  This software interprets entries beginning with '=' as formulas, which are then executed by the spreadsheet software.  The software's formula language often allows methods to access hyperlinks or the local command line, and frequently allows enough characters to invoke an entire script. Attackers can populate data fields which, when saved to a CSV file, may attempt information exfiltration or other malicious activity when automatically executed by the spreadsheet software.
**상태**: Incomplete

---

### 124 - Buffer Underwrite ('Buffer Underflow')
**설명**: The product writes to a buffer using an index or pointer that references a memory location prior to the beginning of the buffer.

This typically occurs when a pointer or its index is decremented to a position before the buffer, when pointer arithmetic results in a position before the beginning of the valid memory location, or when a negative index is used.
**상태**: Incomplete

---

### 126 - Buffer Over-read
**설명**: The product reads from a buffer using buffer access mechanisms such as indexes or pointers that reference memory locations after the targeted buffer.
**상태**: Draft

---

### 1268 - Policy Privileges are not Assigned Consistently Between Control and Data Agents
**설명**: The product's hardware-enforced access control for a particular resource improperly accounts for privilege discrepancies between control and write policies.
			   


				
**상태**: Draft

---

### 127 - Buffer Under-read
**설명**: The product reads from a buffer using buffer access mechanisms such as indexes or pointers that reference memory locations prior to the targeted buffer.

This typically occurs when the pointer or its index is decremented to a position before the buffer, when pointer arithmetic results in a position before the beginning of the valid memory location, or when a negative index is used. This may result in exposure of sensitive information or possibly a crash.
**상태**: Draft

---

### 1298 - Hardware Logic Contains Race Conditions
**설명**: A race condition in the hardware logic results in undermining security guarantees of the system.


				
**상태**: Draft

---

### 13 - ASP.NET Misconfiguration: Password in Configuration File
**설명**: Storing a plaintext password in a configuration file allows anyone who can read the file access to the password-protected resource making them an easy target for attackers.
**상태**: Draft

---

### 131 - Incorrect Calculation of Buffer Size
**설명**: The product does not correctly calculate the size to be used when allocating a buffer, which could lead to a buffer overflow.
**상태**: Draft

---

### 1319 - Improper Protection against Electromagnetic Fault Injection (EM-FI)
**설명**: The device is susceptible to electromagnetic fault injection attacks, causing device internal information to be compromised or security mechanisms to be bypassed.


				
**상태**: Incomplete

---

### 1331 - Improper Isolation of Shared Resources in Network On Chip (NoC)
**설명**: The Network On Chip (NoC) does not isolate or incorrectly isolates its on-chip-fabric and internal resources such that they are shared between trusted and untrusted agents, creating timing channels.


			  
**상태**: Stable

---

### 1334 - Unauthorized Error Injection Can Degrade Hardware Redundancy
**설명**: An unauthorized agent can inject errors into a redundant block to deprive the system of redundancy or put the system in a degraded operating mode.


				
**상태**: Draft

---

### 1390 - Weak Authentication
**설명**: The product uses an authentication mechanism to restrict access to specific users or identities, but the mechanism does not sufficiently prove that the claimed identity is correct.


	
**상태**: Incomplete

---

### 14 - Compiler Removal of Code to Clear Buffers
**설명**: Sensitive memory is cleared according to the source code, but compiler optimizations leave the memory untouched when it is not read from again, aka "dead store removal."


            
**상태**: Draft

---

### 190 - Integer Overflow or Wraparound
**설명**: The product performs a calculation that can
         produce an integer overflow or wraparound when the logic
         assumes that the resulting value will always be larger than
         the original value. This occurs when an integer value is
         incremented to a value that is too large to store in the
         associated representation. When this occurs, the value may
         become a very small or negative number.
**상태**: Stable

---

### 20 - Improper Input Validation
**설명**: The product receives input or data, but it does
        not validate or incorrectly validates that the input has the
        properties that are required to process the data safely and
        correctly.


	   
**상태**: Stable

---

### 219 - Storage of File with Sensitive Data Under Web Root
**설명**: The product stores sensitive data under the web document root with insufficient access control, which might make it accessible to untrusted parties.


	   Besides public-facing web pages and code, products may store sensitive data, code that is not directly invoked, or other files under the web document root of the web server.  If the server is not configured or otherwise used to prevent direct access to those files, then attackers may obtain this sensitive data.
	 
**상태**: Draft

---

### 220 - Storage of File With Sensitive Data Under FTP Root
**설명**: The product stores sensitive data under the FTP server root with insufficient access control, which might make it accessible to untrusted parties.
**상태**: Draft

---

### 24 - Path Traversal: '../filedir'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize "../" sequences that can resolve to a location that is outside of that directory.


            
**상태**: Incomplete

---

### 25 - Path Traversal: '/../filedir'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize "/../" sequences that can resolve to a location that is outside of that directory.


            
**상태**: Incomplete

---

### 250 - Execution with Unnecessary Privileges
**설명**: The product performs an operation at a privilege level that is higher than the minimum level required, which creates new weaknesses or amplifies the consequences of other weaknesses.


            
**상태**: Draft

---

### 258 - Empty Password in Configuration File
**설명**: Using an empty string as a password is insecure.
**상태**: Incomplete

---

### 26 - Path Traversal: '/dir/../filename'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize "/dir/../filename" sequences that can resolve to a location that is outside of that directory.


            
**상태**: Draft

---

### 260 - Password in Configuration File
**설명**: The product stores a password in a configuration file that might be accessible to actors who do not know the password.

This can result in compromise of the system for which the password is used. An attacker could gain access to this file and learn the stored password or worse yet, change the password to one of their choosing.
**상태**: Incomplete

---

### 266 - Incorrect Privilege Assignment
**설명**: A product incorrectly assigns a privilege to a particular actor, creating an unintended sphere of control for that actor.
**상태**: Draft

---

### 267 - Privilege Defined With Unsafe Actions
**설명**: A particular privilege, role, capability, or right can be used to perform unsafe actions that were not intended, even when it is assigned to the correct entity.
**상태**: Incomplete

---

### 268 - Privilege Chaining
**설명**: Two distinct privileges, roles, capabilities, or rights can be combined in a way that allows an entity to perform unsafe actions that would not be allowed without that combination.
**상태**: Draft

---

### 269 - Improper Privilege Management
**설명**: The product does not properly assign, modify, track, or check privileges for an actor, creating an unintended sphere of control for that actor.
**상태**: Draft

---

### 27 - Path Traversal: 'dir/../../filename'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize multiple internal "../" sequences that can resolve to a location that is outside of that directory.


            
**상태**: Draft

---

### 270 - Privilege Context Switching Error
**설명**: The product does not properly manage privileges while it is switching between different contexts that have different privileges or spheres of control.
**상태**: Draft

---

### 271 - Privilege Dropping / Lowering Errors
**설명**: The product does not drop privileges before passing control of a resource to an actor that does not have those privileges.

In some contexts, a system executing with elevated permissions will hand off a process/file/etc. to another process or user. If the privileges of an entity are not reduced, then elevated privileges are spread throughout a system and possibly to an attacker.
**상태**: Incomplete

---

### 272 - Least Privilege Violation
**설명**: The elevated privilege level required to perform operations such as chroot() should be dropped immediately after the operation is performed.
**상태**: Incomplete

---

### 273 - Improper Check for Dropped Privileges
**설명**: The product attempts to drop privileges but does not check or incorrectly checks to see if the drop succeeded.

If the drop fails, the product will continue to run with the raised privileges, which might provide additional access to unprivileged users.
**상태**: Incomplete

---

### 274 - Improper Handling of Insufficient Privileges
**설명**: The product does not handle or incorrectly handles when it has insufficient privileges to perform an operation, leading to resultant weaknesses.
**상태**: Draft

---

### 28 - Path Traversal: '..\filedir'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize "..\" sequences that can resolve to a location that is outside of that directory.


            
**상태**: Incomplete

---

### 280 - Improper Handling of Insufficient Permissions or Privileges 
**설명**: The product does not handle or incorrectly handles when it has insufficient privileges to access resources or functionality as specified by their permissions. This may cause it to follow unexpected code paths that may leave the product in an invalid state.
**상태**: Draft

---

### 285 - Improper Authorization
**설명**: The product does not perform or incorrectly performs an authorization check when an actor attempts to access a resource or perform an action.


            
**상태**: Draft

---

### 287 - Improper Authentication
**설명**: When an actor claims to have a given identity, the product does not prove or insufficiently proves that the claim is correct.
**상태**: Draft

---

### 288 - Authentication Bypass Using an Alternate Path or Channel
**설명**: The product requires authentication, but the product has an alternate path or channel that does not require authentication.
**상태**: Incomplete

---

### 289 - Authentication Bypass by Alternate Name
**설명**: The product performs authentication based on the name of a resource being accessed, or the name of the actor performing the access, but it does not properly check all possible names for that resource or actor.
**상태**: Incomplete

---

### 29 - Path Traversal: '\..\filename'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize '\..\filename' (leading backslash dot dot) sequences that can resolve to a location that is outside of that directory.


            
**상태**: Incomplete

---

### 290 - Authentication Bypass by Spoofing
**설명**: This attack-focused weakness is caused by incorrectly implemented authentication schemes that are subject to spoofing attacks.
**상태**: Incomplete

---

### 291 - Reliance on IP Address for Authentication
**설명**: The product uses an IP address for authentication.

IP addresses can be easily spoofed. Attackers can forge the source IP address of the packets they send, but response packets will return to the forged IP address. To see the response packets, the attacker has to sniff the traffic between the victim machine and the forged IP address. In order to accomplish the required sniffing, attackers typically attempt to locate themselves on the same subnet as the victim machine. Attackers may be able to circumvent this requirement by using source routing, but source routing is disabled across much of the Internet today. In summary, IP address verification can be a useful part of an authentication scheme, but it should not be the single factor required for authentication.
**상태**: Incomplete

---

### 293 - Using Referer Field for Authentication
**설명**: The referer field in HTTP requests can be easily modified and, as such, is not a valid means of message integrity checking.
**상태**: Draft

---

### 294 - Authentication Bypass by Capture-replay
**설명**: A capture-replay flaw exists when the design of the product makes it possible for a malicious user to sniff network traffic and bypass authentication by replaying it to the server in question to the same effect as the original message (or with minor changes).

Capture-replay attacks are common and can be difficult to defeat without cryptography. They are a subset of network injection attacks that rely on observing previously-sent valid commands, then changing them slightly if necessary and resending the same commands to the server.
**상태**: Incomplete

---

### 30 - Path Traversal: '\dir\..\filename'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize '\dir\..\filename' (leading backslash dot dot) sequences that can resolve to a location that is outside of that directory.


            
**상태**: Draft

---

### 301 - Reflection Attack in an Authentication Protocol
**설명**: Simple authentication protocols are subject to reflection attacks if a malicious user can use the target machine to impersonate a trusted user.


		   
**상태**: Draft

---

### 302 - Authentication Bypass by Assumed-Immutable Data
**설명**: The authentication scheme or implementation uses key data elements that are assumed to be immutable, but can be controlled or modified by the attacker.
**상태**: Incomplete

---

### 303 - Incorrect Implementation of Authentication Algorithm
**설명**: The requirements for the product dictate the use of an established authentication algorithm, but the implementation of the algorithm is incorrect.

This incorrect implementation may allow authentication to be bypassed.
**상태**: Draft

---

### 304 - Missing Critical Step in Authentication
**설명**: The product implements an authentication technique, but it skips a step that weakens the technique.

Authentication techniques should follow the algorithms that define them exactly, otherwise authentication can be bypassed or more easily subjected to brute force attacks.
**상태**: Draft

---

### 305 - Authentication Bypass by Primary Weakness
**설명**: The authentication algorithm is sound, but the implemented mechanism can be bypassed as the result of a separate weakness that is primary to the authentication error.
**상태**: Draft

---

### 306 - Missing Authentication for Critical Function
**설명**: The product does not perform any authentication for functionality that requires a provable user identity or consumes a significant amount of resources.
**상태**: Draft

---

### 307 - Improper Restriction of Excessive Authentication Attempts
**설명**: The product does not implement sufficient measures to prevent multiple failed authentication attempts within a short time frame.
**상태**: Draft

---

### 308 - Use of Single-factor Authentication
**설명**: The use of single-factor authentication can lead to unnecessary risk of compromise when compared with the benefits of a dual-factor authentication scheme.

While the use of multiple authentication schemes is simply piling on more complexity on top of authentication, it is inestimably valuable to have such measures of redundancy. The use of weak, reused, and common passwords is rampant on the internet. Without the added protection of multiple authentication schemes, a single mistake can result in the compromise of an account. For this reason, if multiple schemes are possible and also easy to use, they should be implemented and required.
**상태**: Draft

---

### 309 - Use of Password System for Primary Authentication
**설명**: The use of password systems as the primary means of authentication may be subject to several flaws or shortcomings, each reducing the effectiveness of the mechanism.
**상태**: Draft

---

### 31 - Path Traversal: 'dir\..\..\filename'
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize 'dir\..\..\filename' (multiple internal backslash dot dot) sequences that can resolve to a location that is outside of that directory.


            
**상태**: Draft

---

### 313 - Cleartext Storage in a File or on Disk
**설명**: The product stores sensitive information in cleartext in a file, or on disk.

The sensitive information could be read by attackers with access to the file, or with physical or administrator access to the raw disk. Even if the information is encoded in a way that is not human-readable, certain techniques could determine which encoding is being used, then decode the information.
**상태**: Draft

---

### 322 - Key Exchange without Entity Authentication
**설명**: The product performs a key exchange with an actor without verifying the identity of that actor.

Performing a key exchange will preserve the integrity of the information sent between two entities, but this will not guarantee that the entities are who they claim they are. This may enable an attacker to impersonate an actor by modifying traffic between the two entities.  Typically, this involves a victim client that contacts a malicious server that is impersonating a trusted server. If the client skips authentication or ignores an authentication failure, the malicious server may request authentication information from the user. The malicious server can then use this authentication information to log in to the trusted server using the victim's credentials, sniff traffic between the victim and trusted server, etc.
**상태**: Draft

---

### 352 - Cross-Site Request Forgery (CSRF)
**설명**: The web application does not, or cannot, sufficiently verify whether a request was intentionally provided by the user who sent the request, which could have originated from an unauthorized actor. 
**상태**: Stable

---

### 362 - Concurrent Execution using Shared Resource with Improper Synchronization ('Race Condition')
**설명**: The product contains a concurrent code sequence that requires temporary, exclusive access to a shared resource, but a timing window exists in which the shared resource can be modified by another code sequence operating concurrently.


            
**상태**: Draft

---

### 363 - Race Condition Enabling Link Following
**설명**: The product checks the status of a file or directory before accessing it, which produces a race condition in which the file can be replaced with a link before the access is performed, causing the product to access the wrong file.

While developers might expect that there is a very narrow time window between the time of check and time of use, there is still a race condition. An attacker could cause the product to slow down (e.g. with memory consumption), causing the time window to become larger. Alternately, in some situations, the attacker could win the race by performing a large number of attacks.
**상태**: Draft

---

### 364 - Signal Handler Race Condition
**설명**: The product uses a signal handler that introduces a race condition.


            
**상태**: Incomplete

---

### 365 - DEPRECATED: Race Condition in Switch
**설명**: This entry has been deprecated. There are no documented cases in which a switch's control expression is evaluated more than once.

It is likely that this entry was initially created based on a misinterpretation of the original source material. The original source intended to explain how switches could be unpredictable when using threads, if the control expressions used data or variables that could change between execution of different threads. That weakness is already covered by CWE-367. Despite the ambiguity in the documentation for some languages and compilers, in practice, they all evaluate the switch control expression only once. If future languages state that the code explicitly evaluates the control expression more than once, then this would not be a weakness, but the language performing as designed.
**상태**: Deprecated

---

### 366 - Race Condition within a Thread
**설명**: If two threads of execution use a resource simultaneously, there exists the possibility that resources may be used while invalid, in turn making the state of execution undefined.
**상태**: Draft

---

### 367 - Time-of-check Time-of-use (TOCTOU) Race Condition
**설명**: The product checks the state of a resource before using that resource, but the resource's state can change between the check and the use in a way that invalidates the results of the check. This can cause the product to perform invalid actions when the resource is in an unexpected state.

This weakness can be security-relevant when an attacker can influence the state of the resource between check and use. This can happen with shared resources such as files, memory, or even variables in multithreaded programs.
**상태**: Incomplete

---

### 368 - Context Switching Race Condition
**설명**: A product performs a series of non-atomic actions to switch between contexts that cross privilege or other security boundaries, but a race condition allows an attacker to modify or misrepresent the product's behavior during the switch.

This is commonly seen in web browser vulnerabilities in which the attacker can perform certain actions while the browser is transitioning from a trusted to an untrusted domain, or vice versa, and the browser performs the actions on one domain using the trust level and resources of the other domain.
**상태**: Draft

---

### 377 - Insecure Temporary File
**설명**: Creating and using insecure temporary files can leave application and system data vulnerable to attack.
**상태**: Incomplete

---

### 378 - Creation of Temporary File With Insecure Permissions
**설명**: Opening temporary files without appropriate measures or controls can leave the file, its contents and any function that it impacts vulnerable to attack.
**상태**: Draft

---

### 379 - Creation of Temporary File in Directory with Insecure Permissions
**설명**: The product creates a temporary file in a directory whose permissions allow unintended actors to determine the file's existence or otherwise access that file.

On some operating systems, the fact that the temporary file exists may be apparent to any user with sufficient privileges to access that directory. Since the file is visible, the application that is using the temporary file could be known. If one has access to list the processes on the system, the attacker has gained information about what the user is doing at that time. By correlating this with the applications the user is running, an attacker could potentially discover what a user's actions are. From this, higher levels of security could be breached.
**상태**: Incomplete

---

### 403 - Exposure of File Descriptor to Unintended Control Sphere ('File Descriptor Leak')
**설명**: A process does not close sensitive file descriptors before invoking a child process, which allows the child to perform unauthorized I/O operations using those descriptors.

When a new process is forked or executed, the child process inherits any open file descriptors. When the child process has fewer privileges than the parent process, this might introduce a vulnerability if the child process can access the file descriptor but does not have the privileges to access the associated file.
**상태**: Draft

---

### 406 - Insufficient Control of Network Message Volume (Network Amplification)
**설명**: The product does not sufficiently monitor or control transmitted network traffic volume, so that an actor can cause the product to transmit more traffic than should be allowed for that actor.

In the absence of a policy to restrict asymmetric resource consumption, the application or system cannot distinguish between legitimate transmissions and traffic intended to serve as an amplifying attack on target systems. Systems can often be configured to restrict the amount of traffic sent out on behalf of a client, based on the client's origin or access level. This is usually defined in a resource allocation policy. In the absence of a mechanism to keep track of transmissions, the system or application can be easily abused to transmit asymmetrically greater traffic than the request or client should be permitted to.
**상태**: Incomplete

---

### 42 - Path Equivalence: 'filename.' (Trailing Dot)
**설명**: The product accepts path input in the form of trailing dot ('filedir.') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 421 - Race Condition During Access to Alternate Channel
**설명**: The product opens an alternate channel to communicate with an authorized user, but the channel is accessible to other actors.

This creates a race condition that allows an attacker to access the channel before the authorized user does.
**상태**: Draft

---

### 43 - Path Equivalence: 'filename....' (Multiple Trailing Dot)
**설명**: The product accepts path input in the form of multiple trailing dot ('filedir....') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 434 - Unrestricted Upload of File with Dangerous Type
**설명**: The product allows the upload or transfer of dangerous file types that are automatically processed within its environment.
**상태**: Draft

---

### 44 - Path Equivalence: 'file.name' (Internal Dot)
**설명**: The product accepts path input in the form of internal dot ('file.ordir') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 45 - Path Equivalence: 'file...name' (Multiple Internal Dot)
**설명**: The product accepts path input in the form of multiple internal dot ('file...dir') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 46 - Path Equivalence: 'filename ' (Trailing Space)
**설명**: The product accepts path input in the form of trailing space ('filedir ') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 47 - Path Equivalence: ' filename' (Leading Space)
**설명**: The product accepts path input in the form of leading space (' filedir') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 48 - Path Equivalence: 'file name' (Internal Whitespace)
**설명**: The product accepts path input in the form of internal space ('file(SPACE)name') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 49 - Path Equivalence: 'filename/' (Trailing Slash)
**설명**: The product accepts path input in the form of trailing slash ('filedir/') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 502 - Deserialization of Untrusted Data
**설명**: The product deserializes untrusted data without sufficiently ensuring that the resulting data will be valid.
**상태**: Draft

---

### 506 - Embedded Malicious Code
**설명**: The product contains code that appears to be malicious in nature.

Malicious flaws have acquired colorful names, including Trojan horse, trapdoor, timebomb, and logic-bomb. A developer might insert malicious code with the intent to subvert the security of a product or its host system at some time in the future. It generally refers to a program that performs a useful service but exploits rights of the program's user in a way the user does not intend.
**상태**: Incomplete

---

### 528 - Exposure of Core Dump File to an Unauthorized Control Sphere
**설명**: The product generates a core dump file in a directory, archive, or other resource that is stored, transferred, or otherwise made accessible to unauthorized actors.
**상태**: Draft

---

### 529 - Exposure of Access Control List Files to an Unauthorized Control Sphere
**설명**: The product stores access control list files in a directory or other container that is accessible to actors outside of the intended control sphere.

Exposure of these access control list files may give the attacker information about the configuration of the site or system. This information may then be used to bypass the intended security policy or identify trusted systems from which an attack can be launched.
**상태**: Incomplete

---

### 530 - Exposure of Backup File to an Unauthorized Control Sphere
**설명**: A backup file is stored in a directory or archive that is made accessible to unauthorized actors.

Often, older backup files are renamed with an extension such as .~bk to distinguish them from production files. The source code for old files that have been renamed in this manner and left in the webroot can often be retrieved. This renaming may have been performed automatically by the web server, or manually by the administrator.
**상태**: Incomplete

---

### 532 - Insertion of Sensitive Information into Log File
**설명**: The product writes sensitive information to a log file.
**상태**: Incomplete

---

### 533 - DEPRECATED: Information Exposure Through Server Log Files
**설명**: This entry has been deprecated because its abstraction was too low-level.  See CWE-532.
**상태**: Deprecated

---

### 534 - DEPRECATED: Information Exposure Through Debug Log Files
**설명**: This entry has been deprecated because its abstraction was too low-level.  See CWE-532.
**상태**: Deprecated

---

### 538 - Insertion of Sensitive Information into Externally-Accessible File or Directory
**설명**: The product places sensitive information into files or directories that are accessible to actors who are allowed to have access to the files, but not to the sensitive information.
**상태**: Draft

---

### 54 - Path Equivalence: 'filedir\' (Trailing Backslash)
**설명**: The product accepts path input in the form of trailing backslash ('filedir\') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 541 - Inclusion of Sensitive Information in an Include File
**설명**: If an include file source is accessible, the file can contain usernames and passwords, as well as sensitive information pertaining to the application and system.
**상태**: Incomplete

---

### 542 - DEPRECATED: Information Exposure Through Cleanup Log Files
**설명**: This entry has been deprecated because its abstraction was too low-level.  See CWE-532.
**상태**: Deprecated

---

### 544 - Missing Standardized Error Handling Mechanism
**설명**: The product does not use a standardized method for handling errors throughout the code, which might introduce inconsistent error handling and resultant weaknesses.

If the product handles error messages individually, on a one-by-one basis, this is likely to result in inconsistent error handling. The causes of errors may be lost. Also, detailed information about the causes of an error may be unintentionally returned to the user.
**상태**: Draft

---

### 551 - Incorrect Behavior Order: Authorization Before Parsing and Canonicalization
**설명**: If a web server does not fully parse requested URLs before it examines them for authorization, it may be possible for an attacker to bypass authorization protection.

For instance, the character strings /./ and / both mean current directory. If /SomeDirectory is a protected directory and an attacker requests /./SomeDirectory, the attacker may be able to gain access to the resource if /./ is not converted to / before the authorization check is performed.
**상태**: Incomplete

---

### 552 - Files or Directories Accessible to External Parties
**설명**: The product makes files or directories accessible to unauthorized actors, even though they should not be.


	   
**상태**: Draft

---

### 554 - ASP.NET Misconfiguration: Not Using Input Validation Framework
**설명**: The ASP.NET application does not use an input validation framework.
**상태**: Draft

---

### 555 - J2EE Misconfiguration: Plaintext Password in Configuration File
**설명**: The J2EE application stores a plaintext password in a configuration file.

Storing a plaintext password in a configuration file allows anyone who can read the file to access the password-protected resource, making it an easy target for attackers.
**상태**: Draft

---

### 56 - Path Equivalence: 'filedir*' (Wildcard)
**설명**: The product accepts path input in the form of asterisk wildcard ('filedir*') without appropriate validation, which can lead to ambiguous path resolution and allow an attacker to traverse the file system to unintended locations or access arbitrary files.
**상태**: Incomplete

---

### 564 - SQL Injection: Hibernate
**설명**: Using Hibernate to execute a dynamic SQL statement built with user-controlled input can allow an attacker to modify the statement's meaning or to execute arbitrary SQL commands.
**상태**: Incomplete

---

### 566 - Authorization Bypass Through User-Controlled SQL Primary Key
**설명**: The product uses a database table that includes records that should not be accessible to an actor, but it executes a SQL statement with a primary key that can be controlled by that actor.


            
**상태**: Incomplete

---

### 57 - Path Equivalence: 'fakedir/../realdir/filename'
**설명**: The product contains protection mechanisms to restrict access to 'realdir/filename', but it constructs pathnames using external input in the form of 'fakedir/../realdir/filename' that are not handled by those mechanisms. This allows attackers to perform unauthorized actions against the targeted file.
**상태**: Incomplete

---

### 58 - Path Equivalence: Windows 8.3 Filename
**설명**: The product contains a protection mechanism that restricts access to a long filename on a Windows operating system, but it does not properly restrict access to the equivalent short "8.3" filename.

On later Windows operating systems, a file can have a "long name" and a short name that is compatible with older Windows file systems, with up to 8 characters in the filename and 3 characters for the extension. These "8.3" filenames, therefore, act as an alternate name for files with long names, so they are useful pathname equivalence manipulations.
**상태**: Incomplete

---

### 59 - Improper Link Resolution Before File Access ('Link Following')
**설명**: The product attempts to access a file based on the filename, but it does not properly prevent that filename from identifying a link or shortcut that resolves to an unintended resource.
**상태**: Draft

---

### 592 - DEPRECATED: Authentication Bypass Issues
**설명**: This weakness has been deprecated because it covered redundant concepts already described in CWE-287.
**상태**: Deprecated

---

### 593 - Authentication Bypass: OpenSSL CTX Object Modified after SSL Objects are Created
**설명**: The product modifies the SSL context after connection creation has begun.

If the program modifies the SSL_CTX object after creating SSL objects from it, there is the possibility that older SSL objects created from the original context could all be affected by that change.
**상태**: Draft

---

### 603 - Use of Client-Side Authentication
**설명**: A client/server product performs authentication within client code but not in server code, allowing server-side authentication to be bypassed via a modified client that omits the authentication check.

Client-side authentication is extremely weak and may be breached easily. Any attacker may read the source code and reverse-engineer the authentication mechanism to access parts of the application which would otherwise be protected.
**상태**: Draft

---

### 612 - Improper Authorization of Index Containing Sensitive Information
**설명**: The product creates a search index of private or sensitive documents, but it does not properly limit index access to actors who are authorized to see the original information.

Web sites and other document repositories may apply an indexing routine against a group of private documents to facilitate search.  If the index's results are available to parties who do not have access to the documents being indexed, then attackers could obtain portions of the documents by conducting targeted searches and reading the results.  The risk is especially dangerous if search results include surrounding text that was not part of the search query. This issue can appear in search engines that are not configured (or implemented) to ignore critical files that should remain hidden; even without permissions to download these files directly, the remote user could read them.
**상태**: Draft

---

### 616 - Incomplete Identification of Uploaded File Variables (PHP)
**설명**: The PHP application uses an old method for processing uploaded files by referencing the four global variables that are set for each file (e.g. $varname, $varname_size, $varname_name, $varname_type). These variables could be overwritten by attackers, causing the application to process unauthorized files.

These global variables could be overwritten by POST requests, cookies, or other methods of populating or overwriting these variables. This could be used to read or process arbitrary files by providing values such as "/etc/passwd".
**상태**: Incomplete

---

### 619 - Dangling Database Cursor ('Cursor Injection')
**설명**: If a database cursor is not closed properly, then it could become accessible to other users while retaining the same privileges that were originally assigned, leaving the cursor "dangling."

For example, an improper dangling cursor could arise from unhandled exceptions. The impact of the issue depends on the cursor's role, but SQL injection attacks are commonly possible.
**상태**: Incomplete

---

### 639 - Authorization Bypass Through User-Controlled Key
**설명**: The system's authorization functionality does not prevent one user from gaining access to another user's data or record by modifying the key value identifying the data.


            
**상태**: Incomplete

---

### 641 - Improper Restriction of Names for Files and Other Resources
**설명**: The product constructs the name of a file or other resource using input from an upstream component, but it does not restrict or incorrectly restricts the resulting name.

This may produce resultant weaknesses. For instance, if the names of these resources contain scripting characters, it is possible that a script may get executed in the client's browser if the application ever displays the name of the resource on a dynamically generated web page. Alternately, if the resources are consumed by some application parser, a specially crafted name can exploit some vulnerability internal to the parser, potentially resulting in execution of arbitrary code on the server machine. The problems will vary based on the context of usage of such malformed resource names and whether vulnerabilities are present in or assumptions are made by the targeted technology that would make code execution possible.
**상태**: Incomplete

---

### 643 - Improper Neutralization of Data within XPath Expressions ('XPath Injection')
**설명**: The product uses external input to dynamically construct an XPath expression used to retrieve data from an XML database, but it does not neutralize or incorrectly neutralizes that input. This allows an attacker to control the structure of the query.

The net effect is that the attacker will have control over the information selected from the XML database and may use that ability to control application flow, modify logic, retrieve unauthorized data, or bypass important checks (e.g. authentication).
**상태**: Incomplete

---

### 646 - Reliance on File Name or Extension of Externally-Supplied File
**설명**: The product allows a file to be uploaded, but it relies on the file name or extension of the file to determine the appropriate behaviors. This could be used by attackers to cause the file to be misclassified and processed in a dangerous fashion.

An application might use the file name or extension of a user-supplied file to determine the proper course of action, such as selecting the correct process to which control should be passed, deciding what data should be made available, or what resources should be allocated. If the attacker can cause the code to misclassify the supplied file, then the wrong action could occur. For example, an attacker could supply a file that ends in a ".php.gif" extension that appears to be a GIF image, but would be processed as PHP code. In extreme cases, code execution is possible, but the attacker could also cause exhaustion of resources, denial of service, exposure of debug or system data (including application source code), or being bound to a particular server side process. This weakness may be due to a vulnerability in any of the technologies used by the web and application servers, due to misconfiguration, or resultant from another flaw in the application itself.
**상태**: Incomplete

---

### 647 - Use of Non-Canonical URL Paths for Authorization Decisions
**설명**: The product defines policy namespaces and makes authorization decisions based on the assumption that a URL is canonical. This can allow a non-canonical URL to bypass the authorization.


            
**상태**: Incomplete

---

### 648 - Incorrect Use of Privileged APIs
**설명**: The product does not conform to the API requirements for a function call that requires extra privileges. This could allow attackers to gain privileges by causing the function to be called incorrectly.


            
**상태**: Incomplete

---

### 651 - Exposure of WSDL File Containing Sensitive Information
**설명**: The Web services architecture may require exposing a Web Service Definition Language (WSDL) file that contains information on the publicly accessible services and how callers of these services should interact with them (e.g. what parameters they expect and what types they return).


            
**상태**: Incomplete

---

### 652 - Improper Neutralization of Data within XQuery Expressions ('XQuery Injection')
**설명**: The product uses external input to dynamically construct an XQuery expression used to retrieve data from an XML database, but it does not neutralize or incorrectly neutralizes that input. This allows an attacker to control the structure of the query.

The net effect is that the attacker will have control over the information selected from the XML database and may use that ability to control application flow, modify logic, retrieve unauthorized data, or bypass important checks (e.g. authentication).
**상태**: Incomplete

---

### 66 - Improper Handling of File Names that Identify Virtual Resources
**설명**: The product does not handle or incorrectly handles a file name that identifies a "virtual" resource that is not directly specified within the directory that is associated with the file name, causing the product to perform file-based operations on a resource that is not a file.

Virtual file names are represented like normal file names, but they are effectively aliases for other resources that do not behave like normal files. Depending on their functionality, they could be alternate entities. They are not necessarily listed in directories.
**상태**: Draft

---

### 680 - Integer Overflow to Buffer Overflow
**설명**: The product performs a calculation to determine how much memory to allocate, but an integer overflow can occur that causes less memory to be allocated than expected, leading to a buffer overflow.
**상태**: Draft

---

### 689 - Permission Race Condition During Resource Copy
**설명**: The product, while copying or cloning a resource, does not set the resource's permissions or access control until the copy is complete, leaving the resource exposed to other spheres while the copy is taking place.
**상태**: Draft

---

### 73 - External Control of File Name or Path
**설명**: The product allows user input to control or influence paths or file names that are used in filesystem operations.


            
**상태**: Draft

---

### 74 - Improper Neutralization of Special Elements in Output Used by a Downstream Component ('Injection')
**설명**: The product constructs all or part of a command, data structure, or record using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify how it is parsed or interpreted when it is sent to a downstream component.

Software or other automated logic has certain assumptions about what constitutes data and control respectively. It is the lack of verification of these assumptions for user-controlled input that leads to injection problems. Injection problems encompass a wide variety of issues -- all mitigated in very different ways and usually attempted in order to alter the control flow of the process. For this reason, the most effective way to discuss these weaknesses is to note the distinct features that classify them as injection weaknesses. The most important issue to note is that all injection problems share one thing in common -- i.e., they allow for the injection of control plane data into the user-controlled data plane. This means that the execution of the process may be altered by sending code in through legitimate data channels, using no other mechanism. While buffer overflows, and many other flaws, involve the use of some further issue to gain execution, injection problems need only for the data to be parsed.
**상태**: Incomplete

---

### 75 - Failure to Sanitize Special Elements into a Different Plane (Special Element Injection)
**설명**: The product does not adequately filter user-controlled input for special elements with control implications.
**상태**: Draft

---

### 761 - Free of Pointer not at Start of Buffer
**설명**: The product calls free() on a pointer to a memory resource that was allocated on the heap, but the pointer is not at the start of the buffer.


            
**상태**: Incomplete

---

### 762 - Mismatched Memory Management Routines
**설명**: The product attempts to return a memory resource to the system, but it calls a release function that is not compatible with the function that was originally used to allocate that resource.


            
**상태**: Incomplete

---

### 769 - DEPRECATED: Uncontrolled File Descriptor Consumption
**설명**: This entry has been deprecated because it was a duplicate of CWE-774. All content has been transferred to CWE-774.
**상태**: Deprecated

---

### 77 - Improper Neutralization of Special Elements used in a Command ('Command Injection')
**설명**: The product constructs all or part of a command using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the intended command when it is sent to a downstream component.


            
**상태**: Draft

---

### 773 - Missing Reference to Active File Descriptor or Handle
**설명**: The product does not properly maintain references to a file descriptor or handle, which prevents that file descriptor/handle from being reclaimed.

This can cause the product to consume all available file descriptors or handles, which can prevent other processes from performing critical file processing operations.
**상태**: Incomplete

---

### 774 - Allocation of File Descriptors or Handles Without Limits or Throttling
**설명**: The product allocates file descriptors or handles on behalf of an actor without imposing any restrictions on how many descriptors can be allocated, in violation of the intended security policy for that actor.

This can cause the product to consume all available file descriptors or handles, which can prevent other processes from performing critical file processing operations.
**상태**: Incomplete

---

### 775 - Missing Release of File Descriptor or Handle after Effective Lifetime
**설명**: The product does not release a file descriptor or handle after its effective lifetime has ended, i.e., after the file descriptor/handle is no longer needed.

When a file descriptor or handle is not released after use (typically by explicitly closing it), attackers can cause a denial of service by consuming all available file descriptors/handles, or otherwise preventing other system processes from obtaining their own file descriptors/handles.
**상태**: Incomplete

---

### 778 - Insufficient Logging
**설명**: When a security-critical event occurs, the product either does not record the event or omits important details about the event when logging it.


	 
**상태**: Draft

---

### 779 - Logging of Excessive Data
**설명**: The product logs too much information, making log files hard to process and possibly hindering recovery efforts or forensic analysis after an attack.

While logging is a good practice in general, and very high levels of logging are appropriate for debugging stages of development, too much logging in a production environment might hinder a system administrator's ability to detect anomalous conditions. This can provide cover for an attacker while attempting to penetrate a system, clutter the audit trail for forensic analysis, or make it more difficult to debug problems in a production environment.
**상태**: Draft

---

### 78 - Improper Neutralization of Special Elements used in an OS Command ('OS Command Injection')
**설명**: The product constructs all or part of an OS command using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the intended OS command when it is sent to a downstream component.


		   
**상태**: Stable

---

### 785 - Use of Path Manipulation Function without Maximum-sized Buffer
**설명**: The product invokes a function for normalizing paths or file names, but it provides an output buffer that is smaller than the maximum possible size, such as PATH_MAX.

Passing an inadequately-sized output buffer to a path manipulation function can result in a buffer overflow. Such functions include realpath(), readlink(), PathAppend(), and others.
**상태**: Incomplete

---

### 786 - Access of Memory Location Before Start of Buffer
**설명**: The product reads or writes to a buffer using an index or pointer that references a memory location prior to the beginning of the buffer.

This typically occurs when a pointer or its index is decremented to a position before the buffer, when pointer arithmetic results in a position before the beginning of the valid memory location, or when a negative index is used.
**상태**: Incomplete

---

### 788 - Access of Memory Location After End of Buffer
**설명**: The product reads or writes to a buffer using an index or pointer that references a memory location after the end of the buffer.

This typically occurs when a pointer or its index is incremented to a position after the buffer; or when pointer arithmetic results in a position after the buffer.
**상태**: Incomplete

---

### 80 - Improper Neutralization of Script-Related HTML Tags in a Web Page (Basic XSS)
**설명**: The product receives input from an upstream component, but it does not neutralize or incorrectly neutralizes special characters such as "<", ">", and "&" that could be interpreted as web-scripting elements when they are sent to a downstream component that processes web pages.

This may allow such characters to be treated as control characters, which are executed client-side in the context of the user's session. Although this can be classified as an injection problem, the more pertinent issue is the improper conversion of such special characters to respective context-appropriate entities before displaying them to the user.
**상태**: Incomplete

---

### 805 - Buffer Access with Incorrect Length Value
**설명**: The product uses a sequential operation to read or write a buffer, but it uses an incorrect length value that causes it to access memory that is outside of the bounds of the buffer.

When the length value exceeds the size of the destination, a buffer overflow could occur.
**상태**: Incomplete

---

### 806 - Buffer Access Using Size of Source Buffer
**설명**: The product uses the size of a source buffer when reading from or writing to a destination buffer, which may cause it to access memory that is outside of the bounds of the buffer.

When the size of the destination is smaller than the size of the source, a buffer overflow could occur.
**상태**: Incomplete

---

### 836 - Use of Password Hash Instead of Password for Authentication
**설명**: The product records password hashes in a data store, receives a hash of a password from a client, and compares the supplied hash to the hash obtained from the data store.


            
**상태**: Incomplete

---

### 85 - Doubled Character XSS Manipulations
**설명**: The web application does not filter user-controlled input for executable script disguised using doubling of the involved characters.
**상태**: Draft

---

### 862 - Missing Authorization
**설명**: The product does not perform an authorization check when an actor attempts to access a resource or perform an action.
**상태**: Incomplete

---

### 863 - Incorrect Authorization
**설명**: The product performs an authorization check when an actor attempts to access a resource or perform an action, but it does not correctly perform the check.
**상태**: Incomplete

---

### 87 - Improper Neutralization of Alternate XSS Syntax
**설명**: The product does not neutralize or incorrectly neutralizes user-controlled input for alternate script syntax.
**상태**: Draft

---

### 88 - Improper Neutralization of Argument Delimiters in a Command ('Argument Injection')
**설명**: The product constructs a string for a command to be executed by a separate component
in another control sphere, but it does not properly delimit the
intended arguments, options, or switches within that command string.


            
**상태**: Draft

---

### 89 - Improper Neutralization of Special Elements used in an SQL Command ('SQL Injection')
**설명**: The product constructs all or part of an SQL command using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the intended SQL command when it is sent to a downstream component. Without sufficient removal or quoting of SQL syntax in user-controllable inputs, the generated SQL query can cause those inputs to be interpreted as SQL instead of ordinary user data.
**상태**: Stable

---

### 90 - Improper Neutralization of Special Elements used in an LDAP Query ('LDAP Injection')
**설명**: The product constructs all or part of an LDAP query using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the intended LDAP query when it is sent to a downstream component.
**상태**: Draft

---

### 91 - XML Injection (aka Blind XPath Injection)
**설명**: The product does not properly neutralize special elements that are used in XML, allowing attackers to modify the syntax, content, or commands of the XML before it is processed by an end system.

Within XML, special elements could include reserved words or characters such as "<", ">", """, and "&", which could then be used to add new data or modify XML syntax.
**상태**: Draft

---

### 910 - Use of Expired File Descriptor
**설명**: The product uses or accesses a file descriptor after it has been closed.

After a file descriptor for a particular file or device has been released, it can be reused. The code might not write to the original file, since the reused file descriptor might reference a different file or device.
**상태**: Incomplete

---

### 917 - Improper Neutralization of Special Elements used in an Expression Language Statement ('Expression Language Injection')
**설명**: The product constructs all or part of an expression language (EL) statement in a framework such as a Java Server Page (JSP) using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the intended EL statement before it is executed.

Frameworks such as Java Server Page (JSP) allow a developer to insert executable expressions within otherwise-static content. When the developer is not aware of the executable nature of these expressions and/or does not disable them, then if an attacker can inject expressions, this could lead to code execution or other unexpected behaviors.
**상태**: Incomplete

---

### 923 - Improper Restriction of Communication Channel to Intended Endpoints
**설명**: The product establishes a communication channel to (or from) an endpoint for privileged or protected operations, but it does not properly ensure that it is communicating with the correct endpoint.


            
**상태**: Incomplete

---

### 924 - Improper Enforcement of Message Integrity During Transmission in a Communication Channel
**설명**: The product establishes a communication channel with an endpoint and receives a message from that endpoint, but it does not sufficiently ensure that the message was not modified during transmission.

Attackers might be able to modify the message and spoof the endpoint by interfering with the data as it crosses the network or by redirecting the connection to a system under their control.
**상태**: Incomplete

---

### 927 - Use of Implicit Intent for Sensitive Communication
**설명**: The Android application uses an implicit intent for transmitting sensitive data to other applications.


            
**상태**: Incomplete

---

### 93 - Improper Neutralization of CRLF Sequences ('CRLF Injection')
**설명**: The product uses CRLF (carriage return line feeds) as a special element, e.g. to separate lines or records, but it does not neutralize or incorrectly neutralizes CRLF sequences from inputs.
**상태**: Draft

---

### 939 - Improper Authorization in Handler for Custom URL Scheme
**설명**: The product uses a handler for a custom URL scheme, but it does not properly restrict which actors can invoke the handler using the scheme.

Mobile platforms and other architectures allow the use of custom URL schemes to facilitate communication between applications. In the case of iOS, this is the only method to do inter-application communication. The implementation is at the developer's discretion which may open security flaws in the application. An example could be potentially dangerous functionality such as modifying files through a custom URL scheme.
**상태**: Incomplete

---

### 94 - Improper Control of Generation of Code ('Code Injection')
**설명**: The product constructs all or part of a code segment using externally-influenced input from an upstream component, but it does not neutralize or incorrectly neutralizes special elements that could modify the syntax or behavior of the intended code segment.
**상태**: Draft

---

### 940 - Improper Verification of Source of a Communication Channel
**설명**: The product establishes a communication channel to handle an incoming request that has been initiated by an actor, but it does not properly verify that the request is coming from the expected origin.

When an attacker can successfully establish a communication channel from an untrusted origin, the attacker may be able to gain privileges and access unexpected functionality.
**상태**: Incomplete

---

### 941 - Incorrectly Specified Destination in a Communication Channel
**설명**: The product creates a communication channel to initiate an outgoing request to an actor, but it does not correctly specify the intended destination for that actor.


            
**상태**: Incomplete

---

### 95 - Improper Neutralization of Directives in Dynamically Evaluated Code ('Eval Injection')
**설명**: The product receives input from an upstream component, but it does not neutralize or incorrectly neutralizes code syntax before using the input in a dynamic evaluation call (e.g. "eval").
**상태**: Incomplete

---

### 96 - Improper Neutralization of Directives in Statically Saved Code ('Static Code Injection')
**설명**: The product receives input from an upstream component, but it does not neutralize or incorrectly neutralizes code syntax before inserting the input into an executable resource, such as a library, configuration file, or template.
**상태**: Draft

---

### 98 - Improper Control of Filename for Include/Require Statement in PHP Program ('PHP Remote File Inclusion')
**설명**: The PHP application receives input from an upstream component, but it does not restrict or incorrectly restricts the input before its usage in "require," "include," or similar functions.

In certain versions and configurations of PHP, this can allow an attacker to specify a URL to a remote location from which the product will obtain the code to execute. In other cases in association with path traversal, the attacker can specify a local file that may contain executable statements that can be parsed by PHP.
**상태**: Draft

---

### 99 - Improper Control of Resource Identifiers ('Resource Injection')
**설명**: The product receives input from an upstream component, but it does not restrict or incorrectly restricts the input before it is used as an identifier for a resource that may be outside the intended sphere of control.


            
**상태**: Draft

---

### 79 - Improper Neutralization of Input During Web Page Generation ('Cross-site Scripting')
**설명**: The product does not neutralize or incorrectly neutralizes user-controllable input before it is placed in output that is used as a web page that is served to other users.


            
**카테고리**: input_validation
**상태**: Stable

---

### 125 - Out-of-bounds Read
**설명**: The product reads data past the end, or before the beginning, of the intended buffer.
**카테고리**: input_validation
**상태**: Draft

---

### 22 - Improper Limitation of a Pathname to a Restricted Directory ('Path Traversal')
**설명**: The product uses external input to construct a pathname that is intended to identify a file or directory that is located underneath a restricted parent directory, but the product does not properly neutralize special elements within the pathname that can cause the pathname to resolve to a location that is outside of the restricted directory.


            
**카테고리**: file_operations
**상태**: Stable

---

### 23 - Relative Path Traversal
**설명**: The product uses external input to construct a pathname that should be within a restricted directory, but it does not properly neutralize sequences such as ".." that can resolve to a location that is outside of that directory.

This allows attackers to traverse the file system to access files or directories that are outside of the restricted directory.
**카테고리**: file_operations
**상태**: Draft

---

