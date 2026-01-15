---
description: combineLatestWith es un operador de combinación de RxJS que combina el Observable original con los últimos valores de otros Observables para formar un nuevo stream. Es la versión Pipeable Operator de la Creation Function combineLatest, y es ideal para validación de formularios en tiempo real, combinación de datos de múltiples sensores, combinación de filtros de búsqueda y otras situaciones donde se desea integrar los últimos valores de otros streams mientras se transforma o procesa el stream principal.
titleTemplate: ':title | RxJS'
---

# combineLatestWith - Combinar Últimos

El operador `combineLatestWith` combina los **últimos valores** del Observable original y otros Observables especificados en un nuevo stream.
Esta es la versión Pipeable Operator de la Creation Function `combineLatest`.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map } from 'rxjs';

const source1$ = interval(1000); // 0, 1, 2, ...
const source2$ = interval(1500); // 0, 1, 2, ...

source1$
  .pipe(
    combineLatestWith(source2$),
    map(([val1, val2]) => `Stream1: ${val1}, Stream2: ${val2}`)
  )
  .subscribe(console.log);

// Ejemplo de salida:
// Stream1: 0, Stream2: 0
// Stream1: 1, Stream2: 0
// Stream1: 2, Stream2: 0
// Stream1: 2, Stream2: 1
// Stream1: 3, Stream2: 1
// ...
```

- Espera hasta que todos los streams hayan emitido **al menos una vez**, luego emite la combinación del último valor **cada vez que uno de ellos emite**.
- Porque se acepta en forma de tupla, es type-safe en TypeScript.

[🌐 Documentación Oficial de RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 Patrones de Uso Típicos

- **Validación de formularios en tiempo real**: Combinar y validar los últimos valores de múltiples campos
- **Integración de múltiples sensores**: Visualización simultánea de datos de diferentes frecuencias como temperatura, humedad, etc.
- **Filtros de búsqueda combinados**: Integrar selección de categoría e ingreso de palabras clave
- **Vista previa en vivo**: Vista previa en tiempo real combinando múltiples valores de configuración


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de cambio de color (RGB) en tiempo real con múltiples deslizadores.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, combineLatestWith } from 'rxjs';

// Construir la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Ejemplo Práctico de combineLatestWith: Selector de Color RGB</h3>
  <div>
    <label>Rojo: <input type="range" id="red" min="0" max="255" value="128"></label>
    <span id="red-value">128</span>
  </div>
  <div>
    <label>Verde: <input type="range" id="green" min="0" max="255" value="128"></label>
    <span id="green-value">128</span>
  </div>
  <div>
    <label>Azul: <input type="range" id="blue" min="0" max="255" value="128"></label>
    <span id="blue-value">128</span>
  </div>
  <div id="preview" style="width: 200px; height: 100px; border: 1px solid #ccc; margin-top: 10px;"></div>
`;
document.body.appendChild(container);

// Obtener elementos deslizadores
const redSlider = document.getElementById('red') as HTMLInputElement;
const greenSlider = document.getElementById('green') as HTMLInputElement;
const blueSlider = document.getElementById('blue') as HTMLInputElement;

const redValue = document.getElementById('red-value')!;
const greenValue = document.getElementById('green-value')!;
const blueValue = document.getElementById('blue-value')!;
const preview = document.getElementById('preview')!;

// Stream para cada deslizador
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const green$ = fromEvent(greenSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const blue$ = fromEvent(blueSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

// ✅ Versión Pipeable Operator - integrar otros en stream principal
red$
  .pipe(
    combineLatestWith(green$, blue$)
  )
  .subscribe(([r, g, b]) => {
    // Actualizar visualización de valores
    redValue.textContent = String(r);
    greenValue.textContent = String(g);
    blueValue.textContent = String(b);

    // Actualizar color de fondo de vista previa
    preview.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
  });
```

- Mover cualquier deslizador actualizará **inmediatamente** la vista previa con los últimos valores RGB combinados.
- Después de que todos los deslizadores hayan sido manipulados al menos una vez, la última combinación siempre se refleja.


## 🔄 Diferencia con la Creation Function `combineLatest`

### Diferencias Básicas

| | `combineLatest` (Creation Function) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **Ubicación de Uso** | Usado como función independiente | Usado dentro de cadena `.pipe()` |
| **Sintaxis** | `combineLatest([obs1$, obs2$, obs3$])` | `obs1$.pipe(combineLatestWith(obs2$, obs3$))` |
| **Primer Stream** | Trata todos por igual | Trata como stream principal |
| **Ventaja** | Simple y legible | Fácil de combinar con otros operadores |

### Ejemplos de Uso Específicos

**Creation Function Recomendada Solo para Combinación Simple**

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const width$ = fromEvent(window, 'resize').pipe(map(() => window.innerWidth));
const height$ = fromEvent(window, 'resize').pipe(map(() => window.innerHeight));

// Simple y legible
combineLatest([width$, height$]).subscribe(([w, h]) => {
  console.log(`Tamaño de ventana: ${w} x ${h}`);
});
```

**Pipeable Operator Recomendado Cuando Se Agrega Procesamiento de Transformación al Stream Principal**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, throttleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

// ✅ Versión Pipeable Operator - completada en un pipeline
clicks$
  .pipe(
    throttleTime(500),           // Prevenir clics rápidos
    map(() => Date.now()),       // Convertir a timestamp
    startWith(0),                // Establecer valor inicial
    combineLatestWith(timer$),   // Integrar con temporizador
    map(([clickTime, tick]) => ({
      lastClick: clickTime,
      elapsed: tick
    }))
  )
  .subscribe(data => {
    console.log(`Último clic: ${data.lastClick}, Transcurrido: ${data.elapsed} segundos`);
  });

// ❌ Versión Creation Function - se vuelve verbosa
import { combineLatest } from 'rxjs';
combineLatest([
  clicks$.pipe(
    throttleTime(500),
    map(() => Date.now()),
    startWith(0)
  ),
  timer$
]).pipe(
  map(([clickTime, tick]) => ({
    lastClick: clickTime,
    elapsed: tick
  }))
).subscribe(data => {
  console.log(`Último clic: ${data.lastClick}, Transcurrido: ${data.elapsed} segundos`);
});
```

### Resumen

- **`combineLatest`**: Óptimo para simplemente combinar múltiples streams
- **`combineLatestWith`**: Óptimo cuando se desea integrar otros streams mientras se transforma o procesa el stream principal


## ⚠️ Notas Importantes

### Esperar Hasta Que Todos los Streams Emitan al Menos Una Vez

Los valores no se emitirán hasta que todos los Observables hayan emitido al menos una vez.

```ts
import { of, timer } from 'rxjs';
import { combineLatestWith } from 'rxjs';

of(1, 2, 3).pipe(
  combineLatestWith(
    timer(1000),  // Emite después de 1 segundo
  )
).subscribe(console.log);
// Salida: [3, 0]
// * Espera hasta que timer$ emita, luego combina con el último valor (3) de of() en ese momento
```

### Cuidado con las Actualizaciones de Alta Frecuencia

Si cualquiera de los streams se actualiza con frecuencia, el resultado combinado se emitirá con frecuencia en consecuencia.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(100).pipe(
  take(5),
  combineLatestWith(interval(1000).pipe(take(3)))
).subscribe(console.log);
// Salida:
// [0, 0]
// [1, 0]
// [2, 0]
// [3, 0]
// [4, 0]
// [4, 1]
// [4, 2]
```

Controle la frecuencia de actualización con `throttleTime` o `debounceTime` según sea necesario.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, throttleTime, map } from 'rxjs';

const mouseMoves$ = fromEvent(document, 'mousemove').pipe(
  throttleTime(100),  // Limitar cada 100ms
  map(e => ({ x: (e as MouseEvent).clientX, y: (e as MouseEvent).clientY }))
);

const timer$ = interval(1000);

mouseMoves$
  .pipe(combineLatestWith(timer$))
  .subscribe(([pos, tick]) => {
    console.log(`Posición: (${pos.x}, ${pos.y}), Tick: ${tick}`);
  });
```

### Manejo de Errores

Si ocurre un error en cualquier Observable, todo el stream termina con un error.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Ocurrió un error')).pipe(
      catchError(err => of('Error recuperado'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida: [1, 'Error recuperado']
```


## 📚 Operadores Relacionados

- **[combineLatest](/es/guide/creation-functions/combination/combineLatest)** - Versión Creation Function
- **[zipWith](/es/guide/operators/combination/zipWith)** - Emparejar valores correspondientes (orden garantizado)
- **[withLatestFrom](/es/guide/operators/combination/withLatestFrom)** - Combinar solo cuando el stream principal emite
