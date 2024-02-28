# selenium 4
from selenium import webdriver
from selenium.webdriver.chrome.service import Service as ChromeService
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.common.by import By

driver = webdriver.Chrome(service=ChromeService(ChromeDriverManager().install()))
driver.get("https://web.desktroy.io/login/")

username_input = driver.find_element(By.ID, ":r4:")
username_input.send_keys("081274866765")

pass_input = driver.find_element(By.ID, "auth-login-v2-password")
pass_input.send_keys("66765")

submit_btn = driver.find_element(
    By.XPATH, "/div[1]/div/div/div/div/div/div[2]/form/button"
)
submit_btn.click()

while True:
    pass
