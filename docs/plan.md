# Plan de Klimax, un planificador HTN para HDDL 2.1

Estado: H0 en curso (2026-11-23). Nombre: **Klimax**
(decisiones en la sección 7). Este documento sigue el esquema de `plan.md` (Kairos).

---

## 1. Qué es HDDL 2.1 hoy

Klimax soporta **HDDL 1.0 completo y HDDL 2.1** (niveles y semántica en `semantics.md`).

- **HDDL 1.0** (Höller et al., AAAI 2020) es el lenguaje estándar de HTN: tareas abstractas,
  métodos con subtareas ordenadas total o parcialmente, precondiciones de método. Es el lenguaje
  de las competiciones HTN del IPC 2020 y 2023.
- **HDDL 2.1** (Pellier, Albore, Fiorino, Bailon-Ruiz; arXiv 2206.01822 y 2306.07353) **es una
  propuesta, no un estándar cerrado.** Añade acciones durativas, métodos durativos, restricciones
  de duración y de orden sobre los instantes de inicio y fin de las subtareas (álgebra de puntos:
  <, ≤, =, ≠), restricciones de descomposición `(at e φ)` con semántica de PDDL 3.0, fluentes
  numéricos y literales iniciales temporizados.
- Lo que la propuesta deja abierto: la sintaxis definitiva (hay una BNF en LaTeX en
  github.com/pellierd/HDDL2.1), los efectos continuos, los retardos entre subtareas, fechas enteras
  o racionales, axiomas y preferencias, y la no interferencia entre snap actions concurrentes.
- El repositorio de la propuesta trae **solo dos problemas de ejemplo** (Satellite y Transport).
  No hay banco estándar temporal jerárquico, y los propios autores lo dicen.

Consecuencia: quien publique un planificador completo, con una semántica escrita y un banco de
problemas, fija de hecho la referencia. Es una oportunidad mayor que la que tuvo Kairos, y también
un riesgo: hay que decidir semántica donde la propuesta calla, y documentarlo.

## 2. Competidores

Verificado en las páginas del IPC y en los repositorios, salvo donde se indica.

| Planificador | Técnica | Alcance | Limitaciones | Estado / licencia |
|---|---|---|---|---|
| PandaDealer (Olz, Höller, Bercher) | Progresión grounded con detección de callejones sin salida y look-ahead por precondiciones y efectos inferidos | Ganó las tres categorías de orden total del IPC 2023 | Solo orden total; sin tiempo ni números | Sobre pandaPIengine |
| PANDApro / pandaPIengine (Höller) | Progresión grounded; heurísticas clásicas sobre la relajación RC, LM-cut, landmarks; variantes SAT y BDD | Ganó orden parcial óptimo en 2023; segundo en casi todo lo demás | Sin tiempo ni números | BSD-3, C++ |
| Linear Simple / Complex / Lifted (Wu, Olz, Lin, Bercher) | Linealiza el problema de orden parcial y lo resuelve con un planificador de orden total | Ganó orden parcial satisficing y agile en 2023 | Incompleto: una linealización puede perder soluciones | Sobre PANDA |
| TOAD (Höller) | Compila orden total a clásico con autómatas | Mejor configuración individual en 7 dominios de 2023 | Poco fiable fuera de ellos; solo orden total | — |
| Lilotane (Schreiber) | SAT lifted por capas | Segundo en orden total 2020 | Solo orden total; sin efectos condicionales ni existenciales | GPL-3, C++ |
| HyperTensioN (Magnaguagno et al.) | Búsqueda en profundidad lifted, dominio compilado a Ruby | Ganó orden total 2020 | Solo orden total; depende del orden de los métodos | Ruby |
| SIADEX (Granada) | HTN temporal por progresión, lenguaje propio HPDL | Ganó orden parcial 2020; temporal y numérico | Lenguaje propio; flojo en dominios recursivos con mucha ramificación | — |
| Aries (Bit-Monnot) | Crónicas sobre CP/SAT con aprendizaje | Segundo en orden parcial óptimo 2023; HDDL, temporal vía UPF | El planificador clásico del repo figura como «deprecated»; el nuevo, «experimental» | Apache-2.0/MIT, Rust |
| PDDL4J (Pellier, Fiorino) | TFD/PFD, LiftedTreePath, OptiPlan | Participó en 2020 y 2023; tiene el parser de HDDL 2.1 | Sin puestos destacados; el README solo anuncia HDDL 1.0 | LGPL-3, Java |
| SibylSmt (Quenard et al., HPlan 2026) | SAT de orden total + SMT (Z3) para números | 91/140 en su banco numérico, frente a 48 de Aries y 42 de SIADEX | Solo orden total; acotado | Investigación |
| pyHiPOP (Lesire, Albore) | POCL jerárquico | Segundo en orden parcial 2020 | Python, lento | — |
| FAPE, CHIMP, SHOP3 | Temporal jerárquico con lenguajes propios (ANML, etc.) | Referencias históricas | No leen HDDL | *Sin verificar en esta exploración* |

Conclusiones:

1. **Nadie cubre HDDL 2.1.** Los fuertes (PANDA y derivados) son HTN puro, sin tiempo ni números.
   Los que hacen tiempo (SIADEX, Aries, FAPE) usan otro lenguaje o no escalan. El hueco es el
   mismo que encontró Kairos en PDDL temporal.
2. **En HTN puro el listón es alto y está vivo.** PandaDealer y PANDApro son C++ modernos y bien
   publicados. Superarlos en orden total exige igualar sus técnicas y añadir algo; no basta con
   ingeniería.
3. **Nadie es bueno en todo.** Orden total, orden parcial, agile y óptimo los ganan sistemas
   distintos. Un planificador que analice el problema y elija motor, como hace Kairos, puede ganar
   en cobertura global sin ganar cada categoría.
4. **Ninguno es una librería usable.** Son cadenas de ejecutables (parser → grounder → motor) que
   se comunican por ficheros.

## 3. Bancos de problemas y medida

| Banco | Contenido | Uso |
|---|---|---|
| IPC 2020 HTN (`panda-planner-dev/ipc2020-domains`) | 24 dominios de orden total, 9 de orden parcial | Base |
| IPC 2023 HTN (`ipc2023-htn/ipc2023-domains`) | Los de 2020 más SharpSAT, Lamps, Ultralight-Cockpit y Colouring | **Banco principal**; hay resultados y logs oficiales por instancia |
| HTN-Numerical-Benchmarks (Quenard) | 7 familias × 20 instancias, con generadores | Parte numérica |
| HDDL 2.1 (repo de la propuesta) | 2 problemas | Pruebas de sintaxis |
| **Banco de referencia propio** | A crear: niveles L1 a L4, ver 3.1 | Partes numérica y temporal; se publica como contribución |

- Condiciones del IPC: un núcleo, 8 GB, 30 minutos. Puntuación agile
  (1 si se resuelve en 1 s; si no, 1 − log t / log 1800), satisficing (C*/C) y cobertura.
- Como en Kairos: mismos límites para todos, todos los planes verificados, informe a 100 ms, 1 s
  y 30 s. Las cifras oficiales a 30 minutos se citan de los resultados publicados del IPC 2023, y
  se reejecuta a cada competidor en nuestra máquina con los límites cortos.
- Verificación: verificador propio, contrastado con el de pandaPIparser (el que usa el IPC).
- **Objetivo externo: el IPC 2027 tendrá categorías HTN** (organizan Bercher, Yousefi, Med y
  Barták; aún sin web ni fechas). No hubo categoría HTN en 2026. Presentarse da la comparación
  estandarizada que se busca, y su convocatoria de dominios es el sitio natural para proponer el
  banco temporal.

### 3.1 Banco de referencia propio

Además de medirse en los bancos existentes, el proyecto publica un banco de HDDL que cubra los
cuatro niveles (L1 a L4) y pueda servir de referencia. Va en un repositorio aparte, con licencia
abierta, y no depende de Klimax.

| Parte | Contenido |
|---|---|
| Matriz por característica | Cada dominio declara qué usa (orden total o parcial, recursión, números, duraciones, concurrencia requerida, literales temporizados, restricciones de descomposición), para saber qué cubre cada planificador |
| Dominios | Los del IPC extendidos con números y tiempo, los de Quenard, los dos ejemplos de la propuesta y dominios de robótica de PlanSys2. Ninguno hecho a medida de Klimax |
| Generadores | Con semilla y dificultad escalada, para que las instancias se puedan regenerar y ampliar |
| Verificador independiente | De planes jerárquicos numéricos y temporales, según `docs/semantics.md`. Hoy no existe ninguno |
| Soluciones de referencia | Mejor plan conocido por instancia, verificado |
| Resultados | De todo planificador que pueda leer cada nivel, con el arnés publicado y los mismos límites |

Para que sea referencia y no «el banco de Klimax»: la semántica se acuerda con los autores de
HDDL 2.1, el verificador no comparte código de búsqueda con el planificador, y el banco se propone
al taller HPlan y a la convocatoria de dominios del IPC 2027. Que se adopte depende de terceros.

## 4. Diseño propuesto

### 4.1 Relación con Kairos

Planificador nuevo, repositorio propio, C++20, Apache-2.0. Kairos no cambia su comportamiento ni
su rendimiento. Lo que se reutiliza de Kairos, como librería:

- Parser y diagnósticos (HDDL es PDDL más `:task`, `:method`, `:htn`).
- Grounding por alcanzabilidad relajada, invariantes y SAS+.
- Heurísticas clásicas, numéricas y temporales (sobre la relajación del problema HTN).
- STN incremental y validación temporal y numérica de planes.
- Resolver subproblemas: una tarea sin método aplicable, o un objetivo `:goal` de HDDL, se pasa a
  Kairos como problema PDDL.

**Decisión abierta (H0):** hoy Kairos solo exporta la API pública (`kairos.hpp`, `planner.hpp`…).
Para lo anterior hace falta que exporte también sus componentes internos. Opciones:
(a) un segundo destino CMake `kairos::core` con cabeceras internas, sin promesa de estabilidad;
(b) copiar el código. Recomiendo (a): un solo sitio donde arreglar errores, y el único cambio en
Kairos es de empaquetado.

### 4.2 Motores

Un análisis previo clasifica el problema (orden total o parcial, recursivo, con tiempo, con
números, con concurrencia requerida) y elige motor, como en Kairos.

Todas las implementaciones son propias. La columna «Idea de» cita el planificador o el artículo
del que sale la técnica; de ellos no se toma código. El único código reutilizado es el de Kairos,
a través de `kairos::core`.

| Caso | Motor | Idea de |
|---|---|---|
| Orden total, agile | Profundidad lifted con el orden de métodos, memoización y detección de ciclos | HyperTensioN |
| Orden total, general | Progresión grounded con efectos y precondiciones inferidos, look-ahead y poda de callejones | PandaDealer |
| Orden parcial | Primero linealizar y resolver como orden total; si falla, progresión completa de orden parcial | Wu et al.; PANDApro |
| Heurísticas | Relajación RC (el HTN compilado a clásico relajado) evaluada con hFF, landmarks y LM-cut de Kairos; grafo de descomposición de tareas para podar | PANDApro |
| Numérico | Progresión con estados numéricos; poda por intervalos | Kairos |
| Temporal | Snap actions; red temporal sobre inicios y fines de tareas y métodos; compresión cuando el problema es temporalmente simple | Kairos, SIADEX |
| Óptimo | A* con LM-cut sobre RC | PANDApro |

SAT y SMT (Lilotane, SibylSmt) quedan fuera del primer plan: son acotados y arrastran licencias
GPL. Se reconsideran si la medida lo pide.

### 4.3 Requisitos de producto

- Ejecutable y librería, con el mismo estilo de API que Kairos: sin estado global, cancelable,
  anytime con callback, memoria con presupuesto.
- Salida: plan primitivo y árbol de descomposición en el formato del IPC.
- `check` y `validate` como órdenes, con diagnósticos de fichero, línea y columna.
- Módulo Python y motor de unified-planning en cuanto haya HTN puro (H3b): es donde está hoy la
  comunidad activa. El plugin de PlanSys2, después.

## 5. Fases

Leyenda: ✅ hecho · 🔄 en curso · ⏳ pendiente.

| Fase | Estado | Contenido | Criterio de salida |
|---|---|---|---|
| H0 | ✅ | Semántica de HDDL 2.1 por escrito; decisión sobre `kairos::core`; infraestructura de medida con PANDA, Lilotane, HyperTensioN, Aries, SIADEX y PDDL4J instalados | Línea base reproducida frente a los resultados del IPC 2023 |
| H1 | 🔄 | Parser de HDDL 1.0, modelo, grounding con grafo de descomposición, verificador de planes | Lee todo el banco del IPC; verificador de acuerdo con el de PANDA |
| H2 | ⏳ | Motor de orden total (lifted y grounded) | Cobertura ≥ PandaDealer a 1 s y 30 s |
| H3 | ⏳ | Orden parcial: linealización y motor completo | Cobertura ≥ Linear y PANDApro |
| H3b | ⏳ | Módulo Python y motor de unified-planning (`up-klimax`, con `up-kairos` como modelo), para HTN puro | Klimax resuelve desde unified-planning los problemas jerárquicos que su modelo expresa, con los planes verificados |
| H4 | ⏳ | Heurísticas RC, anytime, óptimo | Mejor puntuación satisficing; óptimo ≥ PANDApro |
| H5 | ⏳ | Numérico | Más de 91/140 en el banco de Quenard |
| H6 | ⏳ | Temporal: HDDL 2.1 completo, banco de referencia propio (3.1); Aries, vía unified-planning, como rival | Resuelve el banco; planes validados; comparación con Aries en los problemas que ambos expresan |
| H7 | ⏳ | Robustez, fuzzing, banco por característica, PlanSys2, versión 1.0.0 | Como Kairos |
| H8 | ⏳ | IPC 2027 y artículo | Según el calendario del IPC |

H0 está cerrada: la línea base reproduce el orden del IPC 2023 (`docs/baseline.md`). Objetivo a
batir a 30 s: 494 de 761 en orden total (PandaDealer) y 207 de 283 en orden parcial
(Linear-Simple).

Detalle de H0:

- ✅ Semántica de HDDL 1.0 y HDDL 2.1 por escrito (`docs/semantics.md`), con las decisiones marcadas.
- ✅ `kairos::core` en Kairos (PR #8, fusionado).
- ✅ Repositorio de Klimax con el esqueleto compilando sobre `kairos::core` y sus pruebas.
- ✅ Quince configuraciones de competidores instaladas y verificadas: PANDA (cinco), PandaDealer
  (dos), Linear-Simple, Linear-Complex, TOAD, SIADEX, PDDL4J, Lilotane, HyperTensioN y Aries.
- ✅ Arnés de medida: un núcleo y 8 GB por ejecución, todos los planes verificados.
- ✅ Línea base a 100 ms, 1 s y 30 s en el banco del IPC 2023.
- ✅ Comparación con los resultados publicados del IPC 2023 (el orden coincide).
- ✅ Resultados de la línea base en la documentación de Klimax (`docs/baseline.md`).

Pasan a la fase que los necesita:

- ⏳ Configuraciones óptimas y satisficing con mejora de plan de los competidores (H4).
- ⏳ Banco numérico de Quenard y SibylSmt (H5).
- ⏳ Proponer a los autores de HDDL 2.1 las decisiones de semántica: cuando Klimax resuelva
  HDDL 2.1 completo y haya banco y verificador que lo respalden (tras H6), no antes.

Detalle de H1:

- ✅ Parser de HDDL 1.0 con errores con fichero, línea y columna, sobre el parser de Kairos
  (punto de extensión en Kairos, PR #9).
- ✅ Modelo: tareas, métodos, redes de tareas, restricciones de variables; nombres, aridades,
  tipos y ciclos de orden comprobados.
- ✅ Lee los 1044 problemas del banco del IPC 2023.
- 🔄 Grounding con grafo de descomposición de tareas (`klimax --ground`): un único programa
  Datalog (motor de Kairos) calcula a la vez qué tareas puede pedir la red inicial, qué acciones
  pedidas son alcanzables, qué hechos añaden y qué métodos son descomponibles; Kairos instancia
  después solo esas acciones. Con separación de parámetros. Resuelve 965 de los 1044 problemas en
  menos de 10 s (mediana 14 ms; 884 en menos de 1 s), contiene los 530 planes válidos conocidos y
  demuestra sin solución 3 problemas de Minecraft-Player. Faltan 79 (Snake 24, Minecraft 24,
  Freecell 18, SharpSAT 7, Hiking 5, Satellite 1).
- ✅ Lee los 1116 problemas del banco del IPC 2020.
- ✅ Verificador de planes independiente de la búsqueda, contrastado con el del IPC: de acuerdo
  en 1568 planes reales y en 17 218 de 17 219 variaciones (la restante es un fallo del verificador
  del IPC). Ver `docs/semantics.md`, sección 7.
- ✅ `klimax --check` en el ejecutable.
- ✅ `klimax --verify` en el ejecutable.

H2–H4 son donde está el riesgo: es el terreno de PANDA. H5–H6 son donde está la novedad.

Actividad de los proyectos (noviembre de 2026): el código público de PANDA, Lilotane y PDDL4J
lleva parado desde 2023–2024, y el repositorio de HDDL 2.1 desde enero de 2025. Los proyectos con
desarrollo continuo son Aries y unified-planning. De ahí que la integración con unified-planning
se adelante (H3b) y que el rival en la parte temporal sea Aries, no PANDA.

## 6. Riesgos

- **Semántica no cerrada.** Mitigación: escribirla en H0 con cada decisión marcada, implementarla
  hasta el final y proponerla a los autores con el planificador, el banco y el verificador como
  prueba. Si después cambia algo, se corrige.
- **Sin banco temporal.** Crearlo es trabajo y puede discutirse su imparcialidad. Mitigación:
  derivarlo de dominios del IPC, publicar generadores y proponerlo al IPC 2027.
- **PANDA es un rival serio y activo.** Igualarlo puede llevar más que H2–H4. Mitigación: medir
  desde H0 y no avanzar sin cumplir el criterio.
- **El calendario del IPC 2027 no se conoce.** Si llega antes de H6, presentarse solo en HTN puro.

## 7. Decisiones tomadas (2026-11-23)

1. **Nombre: Klimax** (κλῖμαξ, «escalera»; por la jerarquía de tareas). Libre en PyPI (`klimax`,
   `klimax-planner`), en `PlanSys2/klimax` y en rosdistro; en GitHub solo hay proyectos menores
   ajenos con ese nombre.
2. **Repositorio:** `PlanSys2/klimax`, Apache-2.0, privado hasta la primera versión.
3. **Kairos como librería:** Kairos exporta un destino CMake `kairos::core` con sus cabeceras
   internas, sin promesa de estabilidad. No se copia código.
4. **IPC 2027:** uno de los objetivos, no el único. El calendario del proyecto no depende de él.
5. **Orden de fases: HTN puro primero** (H1–H4), después numérico y temporal. Motivo: el motor
   temporal es el motor HTN más una red temporal; si el motor HTN es débil, lo temporal hereda su
   debilidad y hay que rehacerlo. Además, solo HTN puro tiene banco estándar y rivales fuertes
   contra los que medir, así que es donde se comprueba que el planificador es bueno. La semántica
   temporal se escribe ya en H0 para que el modelo de datos la contemple desde el principio.

## Fuentes

- https://ipc2023-htn.github.io/ y https://ipc2023-htn.github.io/results
- https://ipc2020.hierarchical-task.net/
- https://hierarchical-task.net/ipc
- https://github.com/ipc2023-htn/ipc2023-domains, https://github.com/panda-planner-dev/ipc2020-domains
- https://arxiv.org/abs/2206.01822, https://arxiv.org/abs/2306.07353, https://github.com/pellierd/HDDL2.1
- https://arxiv.org/abs/2609.03938, https://github.com/gaspard-quenard/HTN-Numerical-Benchmarks
- https://github.com/panda-planner-dev/pandaPIengine, https://github.com/panda-planner-dev/pandaPIparser
- https://github.com/domschrei/lilotane, https://github.com/Maumagnaguagno/HyperTensioN
- https://github.com/plaans/aries, https://github.com/pellierd/pddl4j
- https://onlinelibrary.wiley.com/doi/full/10.1002/aaai.12169 (informe del IPC 2023)
