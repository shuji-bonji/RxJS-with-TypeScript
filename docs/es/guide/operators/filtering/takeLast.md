---
description: "takeLast es un operador de filtrado RxJS que muestra sólo los últimos N valores cuando se completa un flujo Observable. Es ideal para situaciones en las que solo se requiere el último valor de todo el flujo, como obtener el último recuento en el registro, mostrar los N valores principales en la tabla de clasificación o el resumen final de datos al finalizar. No se puede utilizar con flujos infinitos, ya que se mantiene en un búfer hasta su finalización."
---

# takeLast - obtiene los últimos N valores

El operador takeLast devuelve sólo los últimos N valores en el momento en que el flujo está **completado**. Mantiene los valores en un buffer hasta que el flujo se completa y los muestra juntos una vez finalizado.

## 🔰 Sintaxis básica y uso

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9
```

**Flujo de operación**:.
1. stream emite 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. internamente mantener últimos 3 en buffer
3. flujo completado 4. valores del buffer 7, 8, 9
4. salida de los valores 7, 8, 9 del búfer en secuencia

[🌐 Documentación oficial de RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contraste con take.

`take` y `takeLast` tienen comportamientos opuestos.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

// take: El primeroNObtener el primero
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Salida: 0, 1, 2(salida inmediata)

// takeLast: Obtener la últimaNObtener el primero
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9(espera a que se complete antes de emitir)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9
---
description: takeLastはObservableストリームが完了した時点で、最後のN個の値のみを出力するRxJSフィルタリングオペレーターです。ログの最新件数取得、リーダーボードの上位N件表示、完了時の最終データサマリーなど、ストリーム全体から最後の値だけが必要な場面に最適です。完了するまでバッファに保持するため無限ストリームでは使用できません。
---


## 💡 Patrón típico de utilización

1. **Obtener las últimas N entradas de registro**.

```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Obtener el último3Obtener los últimos registros
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Salida:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Top de la clasificaciónNRecuperar la parte superior**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Supongamos que ordenados por puntuación
   );

   // Obtener el top3Recuperar el
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Salida: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Resumen final una vez finalizado el procesamiento de datosNResumen de casos**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulación de los datos de los sensores
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Obtener la última5Cálculo de la temperatura media del caso
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`datos${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Última5Adquisición de datos del caso finalizada');
     }
   });
   ```

## 🧠 Ejemplo práctico de código (historial de entradas)

Ejemplo de visualización de los últimos3Este es un ejemplo de visualización de los últimos valores introducidos por el usuario.

```

ts.
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// Creación de elementos UI
const container = document.createElement('div');
document.body.appendChild(contenedor);

const input = document.createElement('input');
input.placeholder = 'Introduce un valor y Enter';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Mostrar historial (últimos 3)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject para guardar los valores de entrada
const inputs$ = new Subject();.

// **IMPORTANTE**: establecer primero la suscripción takeLast
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (valor) => {
    const item = document.createElement('div');
    item.textContent = `- ${valor}`;
    historyDisplay.appendChild(item);
  },.
  complete: () => {
    const nota = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.colour = 'gris';
    note.textContent = '(Vuelve a cargar la página para escribir de nuevo)';
    historyDisplay.appendChild(nota);

    // Desactivar campos de entrada y botones
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Añadir entrada con la tecla Intro
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Añadir: ${input.value}`);
    input.value = '';
  }
});

// Completa con el click del botón y muestra el historial
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>Historia (últimos 3):</strong><br>';
  inputs$.complete(); // flujo completo → takeLast fires
});

```

> [!IMPORTANT]
> **Puntos clave**:
> - `takeLast(3)` Suscribirse al**primero.**debe configurarse primero
> - cuando se pulse el botón `complete()` se dará salida al último de los valores recibidos hasta ese momento.3Se dará salida al último de los valores recibidos hasta ese momento.
> - `complete()` Después de llamar**Después de llamar**a `subscribe` los valores no fluyen.

## ⚠️ Un punto importante a tener en cuenta

> [!WARNING]
> `takeLast` es esperar a que el flujo**Esperar hasta que se complete**Por lo tanto, no funciona con flujos infinitos. Además, la`takeLast(n)` delnes grande, consume mucha memoria.

### 1. No se puede utilizar con flujos infinitos.

`takeLast` no funciona con flujos infinitos porque espera hasta que el flujo se complete.

```

ts.
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Mal ejemplo: usar takeLast con flujos infinitos.
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);.
// No se emite nada (porque el flujo nunca se completa)

```

**Solución.**: `take` Utilice un flujo finito en combinación con

```

ts.
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Buen ejemplo: flujo finito y luego usar takeLast
interval(1000).pipe(
  take(10), // Completa con los 10 primeros
  takeLast(3) // toma los 3 últimos
).subscribe(console.log);.
// Salida: 7, 8, 9

```

### 2. Preste atención al uso de memoria

`takeLast(n)` no funciona con flujos finitos porque retiene la últimanpieza a retener en el buffer,nes grande, consume más memoria.

```

ts.
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Nota: las grandes cantidades de datos se guardan en un búfer
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 registros en memoria
).subscribe(console.log);.

```

## 🎯 last La diferencia entre

```

ts.
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const números$ = range(0, 10);

// last: sólo el último
números$.pipe(
  last()
).subscribe(console.log);
// salida: 9

// takeLast(1): último (salida como valor único, no matriz)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);.
// Salida: 9

// takeLast(3): último 3
números$.pipe(
  takeLast(3)
).subscribe(console.log);
// Salida: 7, 8, 9

```

| operador | Número de adquisiciones | Especificación de la condición | Caso de uso |
|---|---|---|---|
| `last()` | 1Número de | Posible | Obtener la última1Piezas o la última pieza que cumple la condición1Número de |
| `takeLast(n)` | nNúmero de | Imposible | Obtener la últimanObtener simplemente la última pieza que cumple la condición |

## 📋 Uso seguro de tipo

TypeScript Este es un ejemplo de una implementación a prueba de tipos que hace uso de los genéricos en

```

ts.
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interfaz Transacción {
  id: cadena;
  amount: número;
  timestamp: Fecha;
  status: 'pending' | 'completed' | 'failed'; }
}

function getRecentTransactions(
  transactions$: Observable,.
  count: número
): Observable {
  return transacciones$.pipe(
    takeLast(count)
  );
}

// Ejemplo de uso
const transacciones$ = from([.
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const }
  { id: '2', importe: 200, timestamp: new Date('2025-01-02'), estado: 'complete' as const }
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const }
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const }
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const }
] as Transaction[]);.

// Obtener las tres transacciones más recientes
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Salida:.
// 3: 150 yen (pendiente)
// 4: 300 yenes (complete)
// 5: ¥250 (fallido)

```

## 🔄 skip y takeLast combinación de

La parte central del valor se excluye y sólo la últimaNSólo se puede recuperar la última.

```

ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const números$ = range(0, 10); // 0 a 9

// omite los 5 primeros y toma los 3 últimos restantes
números$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // toma los 3 últimos de los 5, 6, 7, 8, 9 restantes
).subscribe(console.log);.
// Salida: 7, 8, 9
```

## 🎓 Resumen

### Cuándo debe usarse takeLast.
- ✅ Si se necesitan los N últimos datos de un flujo.
- ✅ Si se quieren obtener los N últimos logs o transacciones
- ✅ Si se garantiza la finalización del flujo
- ✅ Si desea visualizar un resumen o los N primeros registros de datos

### Cuándo debe utilizar take.
- ✅ Si necesita los N primeros datos del flujo.
- ✅ Si quieres obtener los resultados inmediatamente
- ✅ Si quieres obtener parte de un flujo infinito

### Notas.
- ⚠️ No se puede utilizar con flujos infinitos (ya que no se completan)
- ⚠️ Un n grande en `takeLast(n)` consume memoria
- ⚠️ La salida se compila después de completarse (no inmediatamente)
- ⚠️ A menudo necesita combinarse con `take(n)` para hacer un flujo finito

## 🚀 Siguiente paso.

- **[take](. /take)** - aprende a obtener los primeros n valores.
- **[last](. /last)** - aprende a obtener el último 1 valor.
- skip](. /skip)** - aprende a saltar los N primeros valores.
- filter](. /filter)** - aprende a filtrar en función de condiciones
- **[filtro-operador-casos-prácticos](. /practical-use-cases)** - aprende a utilizar casos de uso reales
