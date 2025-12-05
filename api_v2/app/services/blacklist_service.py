from datetime import datetime, timedelta, timezone
from typing import Optional, Any
import redis
from jose import jwt
import logging

from app.config.settings import settings

logger = logging.getLogger(__name__)


class BlacklistService:
    KEY_PREFIX = "blacklist:"
    
    def __init__(self):
        self.redis_client: Any = None
        try:
            self.redis_client = redis.from_url(
                settings.REDIS_URL,
                decode_responses=True,
                socket_connect_timeout=5,  # 5 second connection timeout
                socket_timeout=5,          # 5 second operation timeout
                retry_on_timeout=True,     # Retry on timeout
                max_connections=10         # Limit connections
            )
            # Test connection on initialization
            self.redis_client.ping()
            self.redis_available = True
            logger.info("Redis connection established successfully")
        except Exception as e:
            logger.warning(f"Redis connection failed: {e}. Blacklist service will operate in degraded mode.")
            self.redis_available = False
    
    def add_to_blacklist(self, jti: str, expires_at: datetime) -> bool:
        """
        Add a JWT ID to the blacklist with expiration time.
        
        Args:
            jti: JWT ID to blacklist
            expires_at: When the token expires (used for Redis TTL)
            
        Returns:
            bool: True if successfully added to blacklist
        """
        if not self.redis_available or not self.redis_client:
            logger.warning("Redis unavailable - skipping blacklist addition")
            return False
            
        try:
            # Calculate TTL in seconds
            ttl = int((expires_at - datetime.now(timezone.utc)).total_seconds())
            
            # Only add if TTL is positive (token not already expired)
            if ttl > 0:
                self.redis_client.setex(f"blacklist:{jti}", ttl, "1")
                logger.info(f"Token {jti} added to blacklist with TTL {ttl}s")
                return True
            return False
        except Exception as e:
            logger.error(f"Error adding to blacklist: {e}")
            return False
    
    def is_blacklisted(self, jti: str) -> bool:
        """
        Check if a JWT ID is in the blacklist.
        
        Args:
            jti: JWT ID to check
            
        Returns:
            bool: True if token is blacklisted
        """
        if not self.redis_available or not self.redis_client:
            logger.warning("Redis unavailable - assuming token is not blacklisted")
            return False
            
        try:
            result = self.redis_client.exists(f"blacklist:{jti}")
            is_blacklisted = bool(result)  # Convert to boolean
            if is_blacklisted:
                logger.info(f"Token {jti} is blacklisted")
            return is_blacklisted
        except Exception as e:
            logger.error(f"Error checking blacklist: {e}")
            # In case of Redis error, assume token is not blacklisted
            # to avoid blocking legitimate users
            return False
    
    def remove_from_blacklist(self, jti: str) -> bool:
        """
        Remove a JWT ID from the blacklist.
        
        Args:
            jti: JWT ID to remove from blacklist
            
        Returns:
            bool: True if successfully removed
        """
        if not self.redis_available or not self.redis_client:
            logger.warning("Redis unavailable - skipping blacklist removal")
            return False
            
        try:
            result = self.redis_client.delete(f"blacklist:{jti}")
            success = bool(result)  # Convert to boolean
            if success:
                logger.info(f"Token {jti} removed from blacklist")
            return success
        except Exception as e:
            logger.error(f"Error removing from blacklist: {e}")
            return False
    
    def get_token_jti(self, token: str) -> Optional[str]:
        """
        Extract JTI from a JWT token.
        
        Args:
            token: JWT token string
            
        Returns:
            str: JTI if found, None otherwise
        """
        try:
            payload = jwt.decode(token, settings.SECRET_KEY, algorithms=["HS256"])
            return payload.get("jti")
        except Exception as e:
            logger.error(f"Error extracting JTI from token: {e}")
            return None
    
    def get_token_expiration(self, token: str) -> Optional[datetime]:
        """
        Extract expiration time from a JWT token.
        
        Args:
            token: JWT token string
            
        Returns:
            datetime: Expiration time if found, None otherwise
        """
        try:
            payload = jwt.decode(token, settings.SECRET_KEY, algorithms=["HS256"])
            exp_timestamp = payload.get("exp")
            if exp_timestamp:
                return datetime.fromtimestamp(exp_timestamp, timezone.utc)
            return None
        except Exception as e:
            logger.error(f"Error extracting expiration from token: {e}")
            return None


# Global instance
blacklist_service = BlacklistService()