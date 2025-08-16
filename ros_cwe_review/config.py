# rag_utils/config.py
import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    """CWE 시스템 설정"""
    
    # MITRE CWE API 설정
    MITRE_CWE_BASE_URL = "https://cwe.mitre.org"
    MITRE_CWE_API_URL = "https://cwe.mitre.org/data/xsd/cwe_latest.xsd"
    MITRE_CWE_DOWNLOAD_URL = "https://cwe.mitre.org/data/xml/cwec_latest.xml.zip"
    
    # NVD API 설정 (CVE 검색용)
    NVD_API_KEY = os.getenv('NVD_API_KEY')
    NVD_API_BASE_URL = os.getenv('NVD_API_BASE_URL', 'https://services.nvd.nist.gov/rest/json')
    NVD_CVE_ENDPOINT = '/cves/2.0'
    
    # API 요청 설정
    NVD_REQUEST_DELAY = int(os.getenv('NVD_REQUEST_DELAY', '1'))
    NVD_MAX_RESULTS_PER_PAGE = int(os.getenv('NVD_MAX_RESULTS_PER_PAGE', '20'))
    
    # ROS 관련 CWE 설정 (실제 존재하는 ID들로 업데이트)
    ROS_CWE_IDS = [
        "CWE-1051", "CWE-1066", "CWE-1080", "CWE-1084", "CWE-1118",
        "CWE-119", "CWE-120", "CWE-121", "CWE-122", "CWE-1223",
        "CWE-1236", "CWE-124", "CWE-126", "CWE-1268", "CWE-127",
        "CWE-1298", "CWE-13", "CWE-131", "CWE-1319", "CWE-1331",
        "CWE-1334", "CWE-1390", "CWE-14", "CWE-190", "CWE-20",
        "CWE-219", "CWE-220", "CWE-24", "CWE-25", "CWE-250",
        "CWE-258", "CWE-26", "CWE-260", "CWE-266", "CWE-267",
        "CWE-268", "CWE-269", "CWE-27", "CWE-270", "CWE-271",
        "CWE-272", "CWE-273", "CWE-274", "CWE-28", "CWE-280",
        "CWE-285", "CWE-287", "CWE-288", "CWE-289", "CWE-29",
        "CWE-290", "CWE-291", "CWE-293", "CWE-294", "CWE-30",
        "CWE-301", "CWE-302", "CWE-303", "CWE-304", "CWE-305",
        "CWE-306", "CWE-307", "CWE-308", "CWE-309", "CWE-31",
        "CWE-313", "CWE-322", "CWE-352", "CWE-362", "CWE-363",
        "CWE-364", "CWE-365", "CWE-366", "CWE-367", "CWE-368",
        "CWE-377", "CWE-378", "CWE-379", "CWE-403", "CWE-406",
        "CWE-42", "CWE-421", "CWE-43", "CWE-434", "CWE-44",
        "CWE-45", "CWE-46", "CWE-47", "CWE-48", "CWE-49",
        "CWE-502", "CWE-506", "CWE-528", "CWE-529", "CWE-530",
        "CWE-532", "CWE-533", "CWE-534", "CWE-538", "CWE-54",
        "CWE-541", "CWE-542", "CWE-544", "CWE-551", "CWE-552",
        "CWE-554", "CWE-555", "CWE-56", "CWE-564", "CWE-566",
        "CWE-57", "CWE-58", "CWE-59", "CWE-592", "CWE-593",
        "CWE-603", "CWE-612", "CWE-616", "CWE-619", "CWE-639",
        "CWE-641", "CWE-643", "CWE-646", "CWE-647", "CWE-648",
        "CWE-651", "CWE-652", "CWE-66", "CWE-680", "CWE-689",
        "CWE-73", "CWE-74", "CWE-75", "CWE-761", "CWE-762",
        "CWE-769", "CWE-77", "CWE-773", "CWE-774", "CWE-775",
        "CWE-778", "CWE-779", "CWE-78", "CWE-785", "CWE-786",
        "CWE-788", "CWE-80", "CWE-805", "CWE-806", "CWE-836",
        "CWE-85", "CWE-862", "CWE-863", "CWE-87", "CWE-88",
        "CWE-89", "CWE-90", "CWE-91", "CWE-910", "CWE-917",
        "CWE-923", "CWE-924", "CWE-927", "CWE-93", "CWE-939",
        "CWE-94", "CWE-940", "CWE-941", "CWE-95", "CWE-96",
        "CWE-98", "CWE-99",
    ]
    
    # ROS 관련 CWE 카테고리 (실제 존재하는 ID들로 업데이트)
    ROS_CWE_CATEGORIES = {
        "authentication": ["CWE-287", "CWE-288", "CWE-289", "CWE-290", "CWE-291", "CWE-293", "CWE-294", "CWE-301", "CWE-302", "CWE-303", "CWE-304", "CWE-305", "CWE-306", "CWE-307", "CWE-308", "CWE-309", "CWE-836"],
        "authorization": ["CWE-285", "CWE-862", "CWE-863", "CWE-250", "CWE-266", "CWE-267", "CWE-268", "CWE-269", "CWE-270", "CWE-271", "CWE-272", "CWE-273", "CWE-274", "CWE-280", "CWE-551", "CWE-566", "CWE-612", "CWE-639", "CWE-648", "CWE-647"],
        "input_validation": ["CWE-20", "CWE-78", "CWE-79", "CWE-89", "CWE-125", "CWE-126", "CWE-127", "CWE-131", "CWE-554", "CWE-564", "CWE-643", "CWE-652", "CWE-74", "CWE-75", "CWE-77", "CWE-80", "CWE-85", "CWE-87", "CWE-88", "CWE-90", "CWE-91", "CWE-94", "CWE-95", "CWE-96", "CWE-98", "CWE-99"],
        "memory_management": ["CWE-119", "CWE-120", "CWE-121", "CWE-122", "CWE-124", "CWE-190", "CWE-680", "CWE-761", "CWE-762", "CWE-786", "CWE-788", "CWE-805", "CWE-806"],
        "file_operations": ["CWE-22", "CWE-23", "CWE-24", "CWE-25", "CWE-26", "CWE-27", "CWE-28", "CWE-29", "CWE-30", "CWE-31", "CWE-42", "CWE-43", "CWE-44", "CWE-45", "CWE-46", "CWE-47", "CWE-48", "CWE-49", "CWE-54", "CWE-56", "CWE-57", "CWE-58", "CWE-59", "CWE-66", "CWE-73", "CWE-377", "CWE-378", "CWE-379", "CWE-403", "CWE-434", "CWE-502", "CWE-528", "CWE-529", "CWE-530", "CWE-532", "CWE-533", "CWE-534", "CWE-538", "CWE-541", "CWE-542", "CWE-552", "CWE-616", "CWE-646", "CWE-651", "CWE-773", "CWE-774", "CWE-775", "CWE-778", "CWE-779", "CWE-785", "CWE-910"],
        "race_conditions": ["CWE-362", "CWE-363", "CWE-364", "CWE-365", "CWE-366", "CWE-367", "CWE-368", "CWE-421", "CWE-689", "CWE-1223", "CWE-1298"],
        "network_security": ["CWE-406", "CWE-923", "CWE-924", "CWE-927", "CWE-940", "CWE-941", "CWE-1051", "CWE-1066", "CWE-1080", "CWE-1084", "CWE-1118", "CWE-1319", "CWE-1331", "CWE-1334", "CWE-1390"],
        "cryptography": ["CWE-313", "CWE-322", "CWE-506", "CWE-917"],
        "logging": ["CWE-532", "CWE-533", "CWE-534", "CWE-538", "CWE-778", "CWE-779"],
        "error_handling": ["CWE-544", "CWE-1118", "CWE-1319", "CWE-1331", "CWE-1334"]
    }
    
    # ROS 컴포넌트별 취약점 매핑 (실제 존재하는 ID들로 업데이트)
    ROS_COMPONENT_CWE_MAPPING = {
        "rclpy/rclcpp": ["CWE-287", "CWE-285", "CWE-434", "CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-502", "CWE-532", "CWE-778"],
        "tf2": ["CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-434", "CWE-532", "CWE-778"],
        "urdf": ["CWE-502", "CWE-20", "CWE-78", "CWE-79", "CWE-434", "CWE-24", "CWE-25", "CWE-26", "CWE-27", "CWE-28", "CWE-29", "CWE-30", "CWE-31"],
        "gazebo": ["CWE-434", "CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-502", "CWE-532", "CWE-778"],
        "moveit": ["CWE-78", "CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-434", "CWE-532", "CWE-778"],
        "rosbag": ["CWE-434", "CWE-502", "CWE-24", "CWE-25", "CWE-26", "CWE-27", "CWE-28", "CWE-29", "CWE-30", "CWE-31", "CWE-532", "CWE-778"],
        "navigation": ["CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-434", "CWE-532", "CWE-778"],
        "control": ["CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-434", "CWE-532", "CWE-778"],
        "diagnostics": ["CWE-532", "CWE-533", "CWE-534", "CWE-538", "CWE-778", "CWE-779"],
        "visualization": ["CWE-79", "CWE-78", "CWE-89", "CWE-20", "CWE-434", "CWE-532", "CWE-778"],
        "hardware_drivers": ["CWE-20", "CWE-119", "CWE-362", "CWE-367", "CWE-434", "CWE-532", "CWE-778", "CWE-1319", "CWE-1331", "CWE-1334"],
        "communication": ["CWE-406", "CWE-923", "CWE-924", "CWE-927", "CWE-940", "CWE-941", "CWE-1051", "CWE-1066", "CWE-1080", "CWE-1084"]
    }
    
    # 데이터베이스 경로
    CWE_DB_PATH = "data/cwe_database"
    RAG_SOURCES_PATH = "data/rag_sources"
    
    @classmethod
    def validate_config(cls):
        """설정 유효성 검사"""
        if not cls.NVD_API_KEY:
            print("NVD API 키가 설정되지 않았습니다. 제한된 요청만 가능합니다.")
        else:
            print(f"NVD API 키가 설정되었습니다: {cls.NVD_API_KEY[:10]}...")
        
        print(f"MITRE CWE URL: {cls.MITRE_CWE_BASE_URL}")
        print(f"NVD CVE API: {cls.NVD_API_BASE_URL}{cls.NVD_CVE_ENDPOINT}")
        print(f"요청 지연: {cls.NVD_REQUEST_DELAY}초")
        print(f"페이지당 결과: {cls.NVD_MAX_RESULTS_PER_PAGE}개")
