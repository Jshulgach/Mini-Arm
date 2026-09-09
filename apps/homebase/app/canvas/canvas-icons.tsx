import {
  MessageCircle,
  CloudSun,
  House,
  Activity,
  Camera,
  CloudRain,
  CloudSnow,
  CloudLightning,
  Sun,
} from 'lucide-react';
export const widgetIcons = {
  chat: MessageCircle,
  weather: CloudSun,
  home: House,
  activity: Activity,
  webcam: Camera,
  gripper: Camera,
};
export function ForecastIcon({
  code,
  size = 42,
}: {
  code: number;
  size?: number;
}) {
  const Icon =
    code >= 95
      ? CloudLightning
      : (code >= 71 && code <= 77) || (code >= 85 && code <= 86)
        ? CloudSnow
        : code >= 51
          ? CloudRain
          : code === 0
            ? Sun
            : CloudSun;
  return <Icon size={size} />;
}
