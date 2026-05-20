---
description: "El operador skipLast es un operador de filtrado de RxJS que omite los últimos N valores del flujo Observable y sólo muestra los valores anteriores."
---

# skipLast - skip los últimos N valores

El operador `skipLast` **salta los últimos N valores emitidos desde el Observable de origen** y sólo emite los valores anteriores. Mantiene los últimos N valores en el buffer hasta que el flujo se completa y emite el resto.

## 🔰 Sintaxis básica y uso

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omite)
```

**Flujo de operación**:.
1. el flujo emite 0, 1, 2, ... emite.
2. mantener los 3 últimos valores (7, 8, 9) en un buffer
3. emite los valores que exceden el tamaño del buffer (0-6)
4. al finalizar el flujo, los valores del búfer (7, 8, 9) no se emiten, sino que se descartan

[🌐 Documentación oficial de RxJS - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Patrón de utilización típico.

- **Excluir últimos datos**: excluir los últimos datos no finalizados.
- **Procesamiento por lotes**: excluir los datos no finalizados antes de que finalice el procesamiento
- Validación de datos**: cuando se requiere la validación de valores posteriores.
- Tratamiento diferido de los datos finalizados**: cuando los últimos N datos no están finalizados.

## 🧠 Ejemplo práctico de código 1: canal de tratamiento de datos

Este es un ejemplo de omisión de los últimos datos no finalizados en el procesamiento de datos.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// UICrear
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Proceso de datos';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Los últimos2casos (datos no finalizados) se omiten y se procesan';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// El flujo de datos (10caso)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// 0.5Publicar datos cada segundo
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Los últimos2Saltar el último caso
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Fijo</strong>
      ID: ${item.id} |
      Valor: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Mostrar elementos omitidos
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Saltar</strong>
        ID: ${item.id} |
        Valor: ${item.value} |
        (Datos no confirmados)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Procesamiento finalizado: ${data.length - 2}Elemento confirmado,2Elementos omitidos`;
    output.appendChild(summary);
  }
});
```

- Los datos se procesan secuencialmente, pero los dos últimos elementos se tratan como no finalizados y se omiten.
- Una vez finalizado, también se muestran los elementos omitidos.

## 🎯 Ejemplo práctico de código 2: Filtrado de registros

Este es un ejemplo de omisión de los últimos registros no finalizados de un flujo de registro.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// UICrear
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Supervisión de registros';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Último3los registros de casos se omiten por estar pendientes de finalización';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Registros confirmados:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Registros pendientes de confirmación (omitidos):</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Registros generados (total)12Registros generados (total,1cada segundo)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'Inicio de sesión del usuario',
      'Inicio de la adquisición de datos',
      'Actualización de la caché',
      'Error de conexión',
      'Reintento de ejecución',
      'Procesamiento de datos finalizado'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Registrar todo (para confirmación)
logs$.subscribe(log => {
  allLogs.push(log);
});

// Los últimos3Mostrar registros confirmados, omitiendo casos
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // Los últimos3Visualizar el caso (registros omitidos)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- Los registros se añaden secuencialmente, pero los tres últimos se omiten por estar pendientes de finalización.
- Una vez finalizados, también se muestran los registros omitidos.

## 🆚 Comparación con operadores similares

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

// skipLast: Los últimosNOmitir un elemento
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6

// takeLast: Los últimosNRecuperar sólo una pieza
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9

// skip: Primera entrada del registroNOmitir un elemento
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Salida: 3, 4, 5, 6, 7, 8, 9
```

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omite)
```

**diferencias visuales**:.

Entrada: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 Saltar]
                                   ^Los últimos3Piezas

takeLast(3): [0~6 Saltar] | 7, 8, 9
                             ^Los últimos3Sólo 1 pieza

skip(3): [0, 1, 2 Saltar] | 3, 4, 5, 6, 7, 8, 9
          ^Primera entrada del registro3Piezas
```

## ⚠️ Notas.

### 1. funciona con flujos infinitos

`skipLast` no funcionará como se pretende con flujos infinitos ya que no puede identificar el último N hasta la finalización.

```

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Mal ejemplo: Con flujos infinitos skipLast con un flujo infinito
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0(3(después de un segundo), 1(4(después de un segundo), 2(5(después de un segundo), ...
// NLa salida continúa indefinidamente con un retardo de 1
// Los últimos3(después de 1,5 segundos), con un retardo de 1,5 segundos.
```

En el caso de flujos infinitos, todos los valores continúan saliendo con un retraso de N porque los últimos N no están determinados. El propósito original de `skipLast` no se logra, ya que no hay un verdadero "último N".

**Solución**: `take` a un flujo finito

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Buen ejemplo: Después de un flujo finito skipLast con un flujo infinito
interval(1000).pipe(
  take(10),      // Primera entrada del registro10Terminado en 1 pieza
  skipLast(3)    // Los últimos3Omitir un elemento
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omite)
```

### 2. prestar atención al tamaño del buffer

`skipLast(n)` siempre mantiene n valores en el buffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 10001 pieza se guarda en un buffer
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Retardo de salida.

skipLast(n)` no muestra nada hasta que n buffers han sido llenados.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Entrada:', val)),
  skipLast(2)
).subscribe(val => console.log('Salida:', val));
// Entrada: 0
// Entrada: 1
// Entrada: 2
// Salida: 0  ← La salida comienza cuando el búfer2La salida comienza cuando el búfer está lleno
// Entrada: 3
// Salida: 1
// Entrada: 4
// Salida: 2
// Finalización (skip3, 4 (omitir)
```

### 4. comportamiento de skipLast(0)

skipLast(0)` no salta nada.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omite)
```

## 💡 Patrones de combinación prácticos

### Patrón 1: Consigue sólo la parte intermedia.

Sáltate el principio y el final y obtén sólo la parte intermedia


### Patrón 2: Validación de datos

Si se requiere la verificación de valores posteriores


```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 se omite)
```

### Patrón 3: Tratamiento de ventanas

Procesamiento de ventana con datos que excluyen los últimos N casos


## 📚 Operadores relacionados

- skip](./skip)** - salta los N primeros valores.
- **[takeLast](./takeLast)** - toma sólo los N últimos valores.
- **[take](./take)** - obtener sólo los N primeros valores.
- skipUntil](./skipUntil)** - salta hasta que se dispara otro Observable.
- skipWhile](./skipWhile)** - saltar mientras se cumple la condición

## Resumen.

El operador `skipLast` salta los últimos N valores del flujo.

- ✅ Ideal cuando no se necesitan los últimos N datos.
- ✅ Útil para excluir datos indeterminados.
- ✅ El tamaño del buffer es sólo N (eficiente en memoria).
- ✅ Se requiere la finalización del flujo
- ⚠️ No disponible para flujos infinitos
- ⚠️ No hay salida hasta que se acumulan N buffers
- ⚠️ A menudo debe combinarse con `take` para flujos finitos
