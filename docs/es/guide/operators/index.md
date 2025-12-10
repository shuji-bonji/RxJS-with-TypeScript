---
description: Los operadores de RxJS se categorizan en siete grupos - transformación, filtrado, combinación, utilidad, condicionales, manejo de errores y multicasting. Aprende el uso práctico de TypeScript con listas completas de operadores y conceptos de pipeline.
---

# Entendiendo los Operadores

Los operadores de RxJS son un conjunto de funciones para transformar, componer y controlar flujos de datos Observable.

Los operadores se suelen usar en combinación con varios otros, y aquí es donde entra en juego el "pipeline".
- [¿Qué es el Pipeline de RxJS?](./pipeline.md)

En RxJS, los operadores se dividen en las siguientes categorías


## Lista de categorías

- [Operadores de Transformación](./transformation/)
- [Operadores de Filtrado](./filtering/)
- [Operadores de Combinación](./combination/)
- [Operadores de Utilidad](./utility/)
- [Operadores Condicionales](./conditional/)
- [Operadores de Manejo de Errores](../error-handling/strategies.md)
- [Operadores de Multicasting](./multicasting/)

Cada categoría contiene varios operadores útiles.
Consulta cada categoría para más detalles.


## Lista de Operadores

Para una descripción detallada de cada operador, haz clic en el enlace correspondiente.

<table style="overflow: visible;">
  <caption>
   Lista de categorías de Operadores
  </caption>
  <thead>
    <tr>
      <th scope="col">Categoría</th>
      <th scope="col">Operador</th>
      <th scope="col">Descripción</th>
    </tr>
  </thead>
  <tbody>
    <!-- Transformation Operators -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Transformación</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Convierte cada valor</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Acumula valores y emite resultados intermedios</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Acumula todos los valores y emite solo el resultado final</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Procesa dos valores consecutivos en pares</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Agrupa flujos por clave</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Ejecución paralela de procesamiento asíncrono</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Ejecuta solo el procesamiento asíncrono más reciente (cancela procesamiento anterior)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Ejecuta procesos asíncronos secuencialmente</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignora nuevos procesos durante la ejecución</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Expande resultados recursivamente</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Publica valores en un array</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Publica valores en intervalos de tiempo especificados</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Publica valores en lotes de un número especificado</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Buffering con condiciones de finalización controladas dinámicamente</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Buffering con control independiente de inicio y fin</td>
    </tr>
    <!-- Filtering Operators -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filtrado</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Solo deja pasar valores que coincidan con la condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>Obtiene solo los primeros N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Obtiene los últimos N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Obtiene valores mientras se cumple la condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Omite los primeros N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Omite los últimos N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Omite valores mientras se satisface la condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Omite valores hasta que otro Observable emita</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Obtiene el primer valor o el primer valor que satisface una condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Obtiene el último valor o el último valor que satisface la condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Obtiene el valor en un índice dado</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Encuentra el primer valor que satisface una condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Obtiene el índice del primer valor que satisface la condición</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Emite el último valor si no se recibe entrada durante un tiempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Deja pasar el primer valor e ignora nuevos valores durante el tiempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Emite el último valor después del tiempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Emite el último valor con Observable personalizado para controlar el período</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Muestrea el último valor en intervalos de tiempo especificados</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignora todos los valores y solo deja pasar completados/errores</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Elimina todos los valores duplicados (emite solo valores únicos)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Elimina valores duplicados consecutivos</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Detecta solo cambios en propiedades específicas de un objeto</td>
    </tr>
    <!-- Combination Operators (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Combinación (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Une otros Observables en secuencia después de completar</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Combina múltiples Observables simultáneamente</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Combina el último valor de cada Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Empareja valores en orden correspondiente</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Adopta solo el primer Observable que emite</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Añade otros últimos valores al flujo principal</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Aplana Higher-order Observables en paralelo</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Aplana Higher-order Observable en secuencia</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Cambia al último Higher-order Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignora nuevo Higher-order Observable durante la ejecución</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Combina los últimos valores de todos los Observables internos</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Empareja los valores correspondientes de cada Observable interno</td>
    </tr>
    <!-- Utility Operators -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utilidad</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Realiza efectos secundarios (ej. salida de log)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Realiza post-procesamiento al completar o con error</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Retrasa todos los valores por un tiempo especificado</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Retrasa cada valor dinámicamente con un Observable separado</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Emite un error si un valor no llega dentro de un tiempo especificado</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Recupera valores hasta que otro Observable emite un valor</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Reintenta hasta un número especificado de veces en caso de error</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Repite un número especificado de veces después de completar</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Añade un valor inicial al comienzo del flujo</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Publica todos los valores juntos en un array</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Convierte una notificación a un objeto Notification</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Convierte el objeto Notification de vuelta a una notificación normal</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Usa el scheduler para controlar cuándo se publican los valores</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Usa el scheduler para controlar cuándo iniciar la suscripción</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Añade una marca de tiempo a cada valor</td>
    </tr>
    <!-- Conditional Operators -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Condicionales</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>Si no hay valor disponible, emite valor por defecto</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Determina si todos los valores satisfacen la condición</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Determina si no se emitió ningún valor</td>
    </tr>
    <!-- Error Handling -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Manejo de Errores</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Captura errores y realiza procesamiento de respaldo</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Reintenta un número especificado de veces en caso de error</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Reintenta con condiciones personalizadas</td>
    </tr>
    <!-- Multicasting -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multicasting</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Comparte Observable entre múltiples suscriptores</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Cachea los últimos N valores y los reproduce para nuevos suscriptores</td>
    </tr>
  </tbody>
</table>
