---
description: "raceWith es un operador join de RxJS que sólo adopta el flujo que emitió el valor primero de entre el Observable original y otros Observables. Es ideal para implementaciones de tiempo de espera, procesos fallback, la adopción más rápida de múltiples fuentes de datos y otras situaciones en las que se requiere la selección por condiciones de carrera; la versión del operador pipeable es conveniente para su uso en pipelines."
---

# raceWith - adopta el flujo más rápido

El operador `raceWith` **adopta** sólo el primer Observable que emite un valor a partir del Observable original y de los otros Observables especificados,
a partir de entonces ignora los demás Observable.
Esta es la versión Pipeable Operator de `race` de Creation Function.

## 🔰 Sintaxis básica y uso

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Despacio. (5Segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2Segundos)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Salida: Rápido (2Segundos)
```

- Sólo el Observable que emitió el valor primero (`fast$` en este ejemplo) es el ganador y continúa con los flujos subsiguientes.
- Otros Observable son ignorados.

[🌐 Documentación oficial de RxJS - raceWith`](https://rxjs.dev/api/operators/raceWith)

## 💡 Patrón de utilización típico.

- **Implementación de tiempo de espera**: competir con el temporizador de tiempo de espera contra el proceso principal.
- **Procesamiento de devolución**: adoptar el más rápido de múltiples fuentes de datos.
- **Interacción con el usuario**: adoptar el más rápido de clic y auto-progreso.

## 🧠 Ejemplos prácticos de código (con interfaz de usuario)

Este es un ejemplo de una carrera entre el clic manual y el temporizador de progreso automático y la adopción de la más rápida.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Creación de la zona de salida
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Ejemplos prácticos de:</h3>';
document.body.appendChild(output);

// Creación de botones
const button = document.createElement('button');
button.textContent = 'Proceder manualmente (5Clic en segundos)';
document.body.appendChild(button);

// Mensaje de espera
const waiting = document.createElement('div');
waiting.textContent = '5Haga clic en el botón en cuestión de segundos o espere al avance automático...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Flujo de clic manual
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Se ha seleccionado el clic manual！')
);

// Temporizador de avance automático (5(después de 2 segundos)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Se ha seleccionado el avance automático！')
);

// Ejecución de la carrera
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- Se adopta el clic manual si se pulsa el botón antes de 5 segundos.
- Después de 5 segundos, se adopta la progresión automática.
- **El más rápido es el ganador**, el más lento se ignora.

## 🔄 Diferencias con Creation Function `race`.

### Diferencias básicas.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Despacio. (5Segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2Segundos)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Salida: Rápido (2Segundos)
```

### Ejemplos concretos de uso.

**Si sólo desea una carrera simple, Creation Function es el camino a seguir**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Servidor1Respuesta de'));
const server2$ = timer(2000).pipe(map(() => 'Servidor2Respuesta de'));
const server3$ = timer(4000).pipe(map(() => 'Servidor3Respuesta de'));

// Simple y fácil de leer
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adoptado:', response);
});
// Salida: Adoptado: Servidor2Respuesta de (Más rápido2Segundos)
```

**Si desea añadir un proceso de conversión a la corriente principal, se recomienda el Pipeable Operator**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Búsqueda';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Corriente principal: Peticiones de búsqueda de los usuarios
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Búsqueda...';

    // APISimular una llamada (3(tarda segundos)
    return timer(3000).pipe(
      map(() => '🔍 Resultados de la búsqueda: 100Visitas por página'),
      catchError((err: unknown) => of('❌ Se ha producido un error'))
    );
  })
);

// ✅ Pipeable OperatorEdición - Completado en un pipeline
userSearch$
  .pipe(
    raceWith(
      // Tiempo de espera (en2segundos)
      timer(2000).pipe(
        map(() => '⏱️ Tiempo de espera (s): La búsqueda está tardando demasiado')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEdición - El flujo principal debe escribirse por separado
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Tiempo de espera (s): La búsqueda está tardando demasiado')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementación del proceso fallback**.

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreación
const output = document.createElement('div');
output.innerHTML = '<h3>Adquisición de datos (con fall-back)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Inicio de la adquisición de datos';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Durante la adquisición...';

  // PrincipalAPI(prioridad)：Hora々Fallo
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ PrincipalAPIAdquisición con éxito de');
      } else {
        return throwError(() => new Error('PrincipalAPIFalla'));
      }
    }),
    catchError((err: unknown) => {
      console.log('PrincipalAPIFallo, cedido a fallback...');
      // Retraso por error, ceder a fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdición - PrincipalAPIAñadir fallback a
  mainApi$
    .pipe(
      raceWith(
        // Copia de seguridadAPI(Fallback)：Ligeramente más lento pero fiable
        timer(2000).pipe(
          map(() => '🔄 Copia de seguridadAPIRecuperado de')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Principal') ? 'green' : 'orange';
      }
    });
});
```

**Adoptar el más rápido de múltiples fuentes de datos**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MúltiplesCDNCarga más rápida desde</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Biblioteca de carga';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Carga de...';

    // CDN1Carga (simulada) desde
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEdición - CDN1principal y otrosCDNcomo competidores.
    return cdn1$.pipe(
      raceWith(
        // CDN2Carga (simulada) desde
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Carga (simulada) desde
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Carga completada</strong><br>
    Adquirido de: ${response.source}<br>
    Archivo: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Resumen.

- **`race`**: ideal si simplemente quieres adoptar el más rápido de múltiples streams.
- **`raceWith`**: ideal si quieres implementar tiempos de espera y fallbacks mientras transformas y procesas el flujo principal

## ⚠️ Notas.

### Ejemplo de implementación de timeout

Implementación del manejo del tiempo de espera usando `raceWith`.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Proceso lento (3segundos)
const slowRequest$ = of('Adquisición de datos con éxito').pipe(delay(3000));

// Tiempo de espera (en2segundos)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Tiempo de espera (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Salida: Tiempo de espera (s)
```

### Todos los flujos están suscritos a

raceWith` suscribe todos los Observable hasta que se decide un ganador.
Una vez decidido el ganador, el Observable perdedor se da de baja automáticamente.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ Disparo')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ Disparo')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Salida:
// fast$ Disparo
// fast
// (slow$es1desabonado al segundo3no se dispara al final del segundo período.)
```

### Para Observable síncrono.

Si todo se emite sincrónicamente, el primero registrado es el ganador.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Salida: A (como se suscribió por primera vez.)
```

## 📚 Operadores relacionados.

- **[race](/es/guide/creation-functions/selection/race)** - Versión de la Creation Function.
- **[timeout](/es/guide/operators/utilidad/timeout)** - Operador de sólo timeout.
- mergeWith](/es/guide/operators/combination/mergeWith)** - Combinar todos los flujos.
