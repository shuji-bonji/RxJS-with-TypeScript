---
description: "auditTime es un operador de filtrado RxJS que espera un tiempo especificado cuando se emite un valor y emite el último valor dentro de ese período. Se utiliza mejor cuando se desea muestrear periódicamente el último estado en eventos de alta frecuencia, como el seguimiento de la posición de desplazamiento, el cambio de tamaño de la ventana, el movimiento del ratón, etc. Es importante entender la diferencia entre este y throttleTime y debounceTime y utilizarlos adecuadamente."
---

# auditTime - último valor emitido después del tiempo especificado

El operador `auditTime` espera un **tiempo especificado** después de que se emita un valor y emite el **último valor** dentro de ese periodo. A continuación, espera a que llegue el siguiente valor.

## 🔰 Sintaxis básica y uso

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Pulsar.！'));
```

**Flujo de operación**:.
1. se produce el primer clic
2. espera 1 segundo (los clics durante este tiempo se registran pero no se emiten)
3. emite el último clic después de 1 segundo
Espere al siguiente clic

[🌐 Documentación oficial de RxJS - `auditTime`](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contraste con throttleTime

`throttleTime` y `auditTime` son similares, pero difieren en los valores que muestran.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Salida primer valor
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Salida.: 0, 4, 8(primer valor de cada periodo)

// auditTime: Salida último valor
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Salida.: 3, 6, 9(último valor de cada periodo)
```

**Comparación de tiempos**:.

```
Fuente:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Primero)   (Primero)   (Primero)

audit:      -------3--------6--------9----|
                  (Último)   (Último)   (Último)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Pulsar.！'));
```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: PrimeroのvalorをSalida
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Salida: 0, 4, 8（各periodoのPrimeroのvalor）

// auditTime: ÚltimoのvalorをSalida
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Salida: 3, 6, 9（各periodoのÚltimoのvalor）
```

## 💡 Patrón típico de utilización

1. **Optimizar el cambio de tamaño de las ventanas**.

```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msObtener el último tamaño del intervalo
   ).subscribe(() => {
     console.log(`Tamaño de la ventana: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Seguimiento de la posición de desplazamiento**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Posición de desplazamiento: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Suavizar el movimiento de arrastre**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Crear elementos arrastrables
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Arrastrar';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Implementar operaciones de arrastre
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Aprox.60FPS(véase también16ms) para actualizar la posición
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```

## 🧠 Ejemplo práctico de código (seguimiento del ratón)

Este ejemplo rastrea los movimientos del ratón y muestra la última posición a intervalos regulares.

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Creación de elementos de interfaz de usuario
const container = document.createElement('div');.
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Por favor, mueva el ratón dentro de esta área';
document.body.appendChild(contenedor);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Evento de movimiento del ratón
fromEvent<MouseEvent>(contenedor, 'moverratón').pipe(
  map(evento => {
    const rect = contenedor.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Obtener la última posición cada 100ms
).subscribe(position => {
  positionDisplay.textContent = `Última posición (cada 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Mover el punto a la última posición
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = `bloque`;
});

```

Este código sólo recuperará y mostrará la última posición cada vez que se mueva el ratón, incluso si el ratón se mueve con frecuencia,100msEl código sólo recupera y muestra la última posición para cada movimiento del ratón.

## 🎯 debounceTime Diferencias entre

`auditTime` y `debounceTime` es que**ambos muestran el último valor**pero el**La temporización es completamente diferente**la salida del último valor.

### La diferencia decisiva

| Operador | operación | utilización del sistema de forma diferente |
|---|---|---|
| `auditTime(ms)` | Cuando entra un valor**msSiempre sale después de**(incluso si la entrada continúa) | Muestreo a intervalos regulares |
| `debounceTime(ms)` | **Después de que la entrada se haya detenido**msSalida después | Esperar a que finalice la entrada |

### Ejemplos concretos：Diferencias en la entrada de búsqueda

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Introducir palabra de búsqueda';
document.body.appendChild(input);

// auditTime: Ejecutar la búsqueda cada 300ms incluso durante la entrada
fromEvent(entrada, 'entrada').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Buscar:', input.value);
});

// debounceTime: espera 300ms después de que se detenga la entrada, luego ejecuta la búsqueda
fromEvent(entrada, 'entrada').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Buscar:', input.value);
});

```

### Diferencias en la línea de tiempo

Diferencia observada cuando un usuario hace clic en "ab'→'abc'→'abcd' al escribir rápidamente:

```

Evento de entrada: a--b--c--d------------|
              ↓
auditTime: ------c-----d----------|
            (después de 300 ms) (después de 300 ms)
            → Búsqueda de 'abc', búsqueda de 'abcd' (2 veces en total)

debounceTime: --------------------d-|
                              (300 ms después de la parada)
            → Búsqueda de "abcd" (sólo una vez en total).

```

**Fácil de recordar**:
- **`auditTime`**: Regularmente auditado (audit)"→ 'Comprobar siempre a intervalos regulares'
- **`debounceTime`**: 'Esperar hasta que se haya calmado (...)'.debounceEspera a que se calme→ 'Esperar hasta que esté tranquilo'

### Uso práctico

```

ts.
// ✅ auditTime si procede.
// - Seguimiento de la posición de desplazamiento (queremos obtenerla periódicamente, aunque estemos desplazándonos todo el tiempo)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // obtener la última posición cada 100ms
).subscribe(/* ... */);

// ✅ si debounceTime es apropiado.
// - caja de búsqueda (queremos buscar después de que se complete la entrada)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // espera 300ms después de que la entrada se detenga
).subscribe(/* ... */);

```

## 📋 Uso seguro

TypeScript Este es un ejemplo de una implementación de tipo seguro que hace uso de los genéricos en

```

ts.
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interfaz MousePosition {
  x: número
  y: número
  timestamp: number; }
}

function trackMousePosition(
  elemento: HTMLElement,.
  intervalMs: número
): Observable {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),.
    map(evento => ({
      x: evento.clienteX, evento.
      y: evento.clienteY,.
      timestamp: Date.now())
    } as MousePosition))
  );
}

// Ejemplo de uso
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px negro sólido';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Posición: (${position.x}, ${position.y}) at ${position.timestamp}`);
});

```

## 🔄 auditTime y throttleTime Combinación de

En ciertos escenarios, ambos pueden combinarse.

```

ts.
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const fuente$ = interval(100).pipe(take(50));.

// orden de throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // pasa el primer valor cada segundo
  auditTime(500) // luego esperar 500ms y emitir el último valor
).subscribe(console.log);.

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Pulsar.！'));
---
description: auditTimeは値が発行されたら指定時間待機し、その期間内の最後の値を出力するRxJSフィルタリングオペレーターです。スクロール位置の追跡、ウィンドウリサイズ、マウス移動などの高頻度イベントで最新の状態を定期的にサンプリングしたい場合に最適です。throttleTimeやdebounceTimeとの違いを理解して適切に使い分けることが重要です。
---


ts.
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// Crear un campo de entrada de búsqueda
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Buscar...' ;
document.body.appendChild(input);

// ❌ Mal ejemplo: utilizar auditTime para la entrada de búsqueda
fromEvent(input, 'input').pipe(
  auditTime(300) // la búsqueda se realiza cada 300ms mientras se introduce la entrada
).subscribe(() => {
  console.log('Búsqueda ejecutada');
});

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Pulsar.！'));
```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('クリック！'));
```

ts.
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Crear un campo de entrada de búsqueda
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Buscar...' ;
document.body.appendChild(input);

// ✅ Buen ejemplo: utilizar debounceTime para la entrada de búsqueda
fromEvent(entrada, 'entrada').pipe(
  debounceTime(300) // Espera 300ms después de que la entrada se detenga antes de buscar
).subscribe(() => {
  console.log('Búsqueda ejecutada', input.value);
});
```

## 🎓 Resumen

### Cuándo se debe utilizar auditTime.
- ✅ Cuando se requieren valores actualizados a intervalos regulares.
- ✅ Eventos de alta frecuencia como desplazamiento, cambio de tamaño, movimiento del ratón.
- ✅ Cuando se requiere un muestreo periódico.
- ✅ Cuando se desea reflejar el estado más reciente.

### Cuando se debe utilizar throttleTime.
- ✅ Cuando se requiere una respuesta inmediata.
- ✅ Si se desea iniciar el procesamiento con el primer valor.
- ✅ Prevención del machaqueo de botones.

### Cuándo utilizar debounceTime.
- ✅ Si desea esperar a la finalización de la entrada.
- ✅ Búsqueda, autocompletar
- ✅ Esperar a que el usuario deje de escribir.

### Notas.
- ⚠️ `auditTime` emite sólo el último valor del periodo (los valores intermedios se descartan).
- ⚠️ No es muy efectivo si se fija para intervalos cortos
- ⚠️ `throttleTime` o `debounceTime` pueden ser más apropiados dependiendo de la aplicación

## 🚀 Próximos pasos.

- **[throttleTime](. /throttleTime)** - aprende a pasar el primer valor.
- **[debounceTime](. /debounceTime)** - aprende cómo emitir valores después de que se detenga la entrada.
- filter](. /filter)** - aprende a filtrar basándose en condiciones.
- **[filtro-operador-casos-prácticos](. /practical-use-cases)** - aprende a utilizar casos de uso reales
