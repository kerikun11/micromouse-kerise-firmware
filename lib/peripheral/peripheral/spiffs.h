/**
 * @file spiffs.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief SPIFFS utility
 * @date 2021-11-21
 * @copyright Copyright (c) 2021 Ryotaro Onuki
 */
#pragma once

#include <esp_spiffs.h>
#include <sys/dirent.h>
#include <sys/stat.h>

namespace peripheral {

class SPIFFS {
public:
  static bool init(const char *mount_path = "/spiffs") {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = mount_path,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
        std::printf("Failed to mount or format SPIFFS\n");
      } else if (ret == ESP_ERR_NOT_FOUND) {
        std::printf("Failed to find SPIFFS partition\n");
      }
      return false;
    }
    return true;
  }
  static bool deinit() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_vfs_spiffs_unregister(NULL));
    return true;
  }
  static void list_dir(const char *name) {
    DIR *dir;
    struct dirent *entry;
    struct stat buf;
    if (!(dir = opendir(name)))
      return;
    while ((entry = readdir(dir)) != NULL) {
      char path[256 + 1];
      snprintf(path, sizeof(path), "%s/%s", name, entry->d_name);
      if (entry->d_type == DT_DIR) {
        printf("          %s/\n", path);
        list_dir(path); // recursive call
      } else {
        stat(path, &buf);
        printf("%9ld %s/%s\n", buf.st_size, name, entry->d_name);
      }
    }
    closedir(dir);
  }
};

}; // namespace peripheral
