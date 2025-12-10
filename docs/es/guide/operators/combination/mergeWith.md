---
description: "mergeWith Pipeable Operator se suscribe a múltiples Observables simultáneamente y los fusiona en paralelo: Ideal para integrar múltiples fuentes de eventos"
titleTemplate: ':title'
---

# mergeWith - Fusionar Múltiples Streams Simultáneamente Dentro de un Pipeline

El operador `mergeWith` **se suscribe simultáneamente** al Observable original y a los otros Observables especificados,
y fusiona los valores emitidos desde cada uno en tiempo real.
Esta es la versión Pipeable Operator de la Creation Function `merge`.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Ejemplo de salida:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Todos los Observables se suscriben simultáneamente, y los valores fluyen **en el orden en que se emiten**.
- No hay garantía de orden, y **depende del tiempo de emisión de cada Observable**.

[🌐 Documentación Oficial de RxJS - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Patrones de Uso Típicos

- **Integrar múltiples fuentes de eventos**: Combinar operaciones de usuario y actualizaciones automáticas
- **Fusionar obtenciones de datos paralelas**: Agregar respuestas de múltiples APIs en un solo stream
- **Fusionar actualizaciones en tiempo real**: Integrar WebSocket y polling


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de integración de eventos de clic de usuario y temporizador de actualización automática para mostrar notificaciones.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo Práctico de mergeWith:</h3>';
document.body.appendChild(output);

// Crear botón
const button = document.createElement('button');
button.textContent = 'Actualización Manual';
document.body.appendChild(button);

// Stream de clics
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Actualización manual ejecutada')
);

// Temporizador de actualización automática (cada 5 segundos)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Actualización automática #${val + 1}`),
  take(3)
);

// Integrar ambos y mostrar
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Al hacer clic en el botón, se muestra inmediatamente la actualización manual,
- Las actualizaciones automáticas también se ejecutan en paralelo cada 5 segundos.
- Ambos eventos se integran en tiempo real.


## 🔄 Diferencia con la Creation Function `merge`

### Diferencias Básicas

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Ubicación de Uso** | Usado como función independiente | Usado dentro de cadena `.pipe()` |
| **Sintaxis** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **Primer Stream** | Trata todos por igual | Trata como stream principal |
| **Ventaja** | Simple y legible | Fácil de combinar con otros operadores |

### Ejemplos de Uso Específicos

**Creation Function Recomendada Solo para Fusión Simple**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Clic'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Movimiento de ratón'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Tecla presionada'));

// Simple y legible
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Salida: Mostrar en el orden en que ocurre cualquier evento
```

**Pipeable Operator Recomendado Cuando Se Agrega Procesamiento de Transformación al Stream Principal**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // Cada 30 segundos

// ✅ Versión Pipeable Operator - completada en un pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // Prevenir clics rápidos
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // Solo dentro de 1 minuto
  )
  .subscribe(event => {
    console.log(`${event.source} actualización: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Versión Creation Function - se vuelve verbosa
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`${event.source} actualización: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**Cuando Se Integran Múltiples Fuentes de Datos**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// Crear botón
const saveButton = document.createElement('button');
saveButton.textContent = 'Guardar';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream principal: Operación de guardado del usuario
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 Guardado manual')
);

// ✅ Versión Pipeable Operator - agregar auto-guardado al stream principal
manualSave$
  .pipe(
    startWith('📝 Edición iniciada'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ Auto-guardado'))  // Auto-guardado cada 10 segundos
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### Resumen

- **`merge`**: Óptimo para simplemente fusionar múltiples streams en igualdad de condiciones
- **`mergeWith`**: Óptimo cuando se desea fusionar otros streams mientras se transforma o procesa el stream principal


## ⚠️ Notas Importantes

### Tiempo de Completación

El stream fusionado no completará hasta que todos los Observables completen.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← Agregar esto nunca completará
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Completo')
});
// Salida: 1 → 2 → 3 → 0 → 1 → ✅ Completo
```

### Controlar el Número de Ejecuciones Concurrentes

Por defecto, todos los streams se ejecutan concurrentemente, pero se pueden controlar en combinación con `mergeMap`.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // Ejecutar hasta 2 concurrentemente
  )
).subscribe(console.log);
```

### Manejo de Errores

Si ocurre un error en cualquier Observable, todo el stream termina con un error.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Ocurrió un error')).pipe(
      catchError(err => of('Error recuperado'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida: 0 → Error recuperado → 1
```


## 📚 Operadores Relacionados

- **[merge](/es/guide/creation-functions/combination/merge)** - Versión Creation Function
- **[concatWith](/es/guide/operators/combination/concatWith)** - Versión Pipeable para combinación secuencial
- **[mergeMap](/es/guide/operators/transformation/mergeMap)** - Mapear cada valor en paralelo
