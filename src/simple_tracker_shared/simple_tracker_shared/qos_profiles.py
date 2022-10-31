from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.qos import QoSPresetProfiles, QoSDurabilityPolicy, QoSLivelinessPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks

#https://github.com/ros2/demos/blob/rolling/quality_of_service_demo/rclpy/quality_of_service_demo_py/incompatible_qos.py

#publisher_callbacks = PublisherEventCallbacks(incompatible_qos=lambda event: get_logger('Talker').info(str(event)))
#subscription_callbacks = SubscriptionEventCallbacks(incompatible_qos=lambda event: get_logger('Listener').info(str(event)))
#subscription_callbacks = SubscriptionEventCallbacks(message_lost=self._message_lost_event_callback)
#subscription_callbacks = SubscriptionEventCallbacks(liveliness=lambda event: get_logger('Listener').info(str(event)))

def get_config_publisher_qos_profile() -> QoSProfile:
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    return qos_profile

    #qos_profile_publisher.durability = QoSDurabilityPolicy.VOLATILE
    #qos_profile_subscription.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    
    #qos_profile_publisher.deadline = Duration(seconds=2)
    #qos_profile_subscription.deadline = Duration(seconds=1)
    
    #qos_profile_publisher.liveliness = QoSLivelinessPolicy.AUTOMATIC
    #qos_profile_subscription.liveliness = QoSLivelinessPolicy.MANUAL_BY_TOPIC
    
    #qos_profile_publisher.liveliness_lease_duration = Duration(seconds=2)
    #qos_profile_subscription.liveliness_lease_duration = Duration(seconds=1)

    #qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT
    #qos_profile_subscription.reliability = QoSReliabilityPolicy.RELIABLE

    #qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    # lifespan=Duration(seconds=300),  liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Duration(seconds=2.2))

    #return QoSPresetProfiles.SYSTEM_DEFAULT.value    

def get_config_subscriber_qos_profile() -> QoSProfile:
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    return qos_profile

    #qos_profile_publisher.durability = QoSDurabilityPolicy.VOLATILE
    #qos_profile_subscription.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    
    #qos_profile_publisher.deadline = Duration(seconds=2)
    #qos_profile_subscription.deadline = Duration(seconds=1)
    
    #qos_profile_publisher.liveliness = QoSLivelinessPolicy.AUTOMATIC
    #qos_profile_subscription.liveliness = QoSLivelinessPolicy.MANUAL_BY_TOPIC
    
    #qos_profile_publisher.liveliness_lease_duration = Duration(seconds=2)
    #qos_profile_subscription.liveliness_lease_duration = Duration(seconds=1)

    #qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT
    #qos_profile_subscription.reliability = QoSReliabilityPolicy.RELIABLE

    #qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    # lifespan=Duration(seconds=300),  liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Duration(seconds=2.2))

    #return QoSPresetProfiles.SYSTEM_DEFAULT.value    

def get_topic_publisher_qos_profile(reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE) -> QoSProfile:
    #return QoSPresetProfiles.SENSOR_DATA.value    
    #return QoSPresetProfiles.SYSTEM_DEFAULT.value    
    qos_profile = QoSProfile(depth=5)
    qos_profile.reliability = reliability
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    return qos_profile

def get_topic_subscriber_qos_profile(reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE) -> QoSProfile:
    #return QoSPresetProfiles.SENSOR_DATA.value    
    #return QoSPresetProfiles.SYSTEM_DEFAULT.value    
    qos_profile = QoSProfile(depth=5)
    qos_profile.reliability = reliability
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    return qos_profile

def get_service_qos_profile() -> QoSProfile:
    return QoSPresetProfiles.SERVICES_DEFAULT.value