Для g_unit = 128 (угол примерно 1.5 градуса) среднее время работы препроцессинга на 3-dof 79'894.44ms, то есть почти 80s.
Для g_unit = 64  (угол примерно 3.0 градуса) среднее время работы препроцессинга на 3-dof 45'107.88ms
Для g_unit = 32  (угол примерно 6.0 градуса) среднее время работы препроцессинга на 3-dof  6'202.34ms

По памяти 2-dof препроцессинг занимает 800'000 bytes (~780KB), lazy A* до 600'000 bytes
Память на 3-dof 435MB против 50MB

На 4-dof для 6.0 градуса 413MB памяти препроцессингом
Для 5-dof памяти не хватило
