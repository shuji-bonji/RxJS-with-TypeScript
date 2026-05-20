---
description: "combineLatestWith es un operador de combinación de RxJS que combina el Observable original con los últimos valores de otros Observables. Es ideal para situaciones en las que se desea supervisar constantemente los últimos valores de varios flujos dependientes, como la validación en tiempo real de entradas de formularios, la sincronización de varios estados, las actualizaciones en tiempo real de resultados de cálculos, etc. La versión del operador pipeable es conveniente para su uso dentro de tuberías."
---

# combineLatestWith - combina los últimos valores

El operador combineLatestWith combina todos los **últimos valores** del Observable original y de cualquier otro Observable especificado.
Cada vez que se emite un nuevo valor desde uno de los Observable, se emite un resultado que combina todos los últimos valores.
Se trata de la versión Pipeable Operator de "combineLatest" de Creation Function.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Ejemplos de salida:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Después de que cada Observable haya emitido **al menos un valor**, se emite el valor combinado.
- Cada vez que entra un nuevo valor por cualquier lado, se vuelve a emitir el último par.

[🌐 Documentación oficial de RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Patrón de utilización típico.

- **Validación en tiempo real de entradas de formulario**: monitorización constante del último estado de múltiples campos.
- **Sincronización de múltiples estados dependientes**: combinación de valores de configuración y entradas del usuario
- **Actualización en tiempo real de los resultados de los cálculos**: cálculo inmediato de los resultados a partir de múltiples valores de entrada

## 🧠 Ejemplos prácticos de código (con interfaz de usuario)

Ejemplo de cálculo del importe total en tiempo real a partir de entradas de precio y cantidad.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Creación de áreas de salida
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Ejemplos prácticos de:</h3>';
document.body.appendChild(output);

// Creación de campos de entrada
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Precio unitario';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Cantidad';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Área de visualización de resultados
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// de cada entradaObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Calculado combinando los últimos valores
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Importe total: ¥${total.toLocaleString()}</strong>`;
  });
```

- Al introducir cualquiera de los dos campos, el total se **recalcula inmediatamente** a partir de los dos valores más recientes.
- Se utiliza `startWith()` para obtener el resultado combinado desde el principio.

## 🔄 Diferencias con la Creation Function `combineLatest`.

### Diferencias básicas.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Ejemplos de salida:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Ejemplos concretos de uso.

**Si sólo desea combinaciones simples, Creation Function es el camino a seguir**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Sencillo y fácil de leer
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Edad)`);
});
// Salida: Yamada Taro (30Edad)
```

**Si desea añadir un proceso de conversión al flujo principal, se recomienda el Pipeable Operator**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Buscar...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Todos</option><option>Libros</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Corriente principal: Buscar palabras clave
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Después de la entrada300msEsperar
  startWith('')
);

// Subflujo: Selección de categoría
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Todos')
);

// ✅ Pipeable OperatorEdición - Completo en una sola cadena
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Convertido a minúsculas
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Buscar: "${result.term}" Categoría: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionEdición - Convertir en redundante
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Buscar: "${result.term}" Categoría: ${result.category} [${result.timestamp}]`;
});
```

**Al combinar varios valores de configuración**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Creación de slider
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Corriente principal: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorEdición - RedCombinar otros colores como principal
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Resumen.

- **`combineLatest`**: ideal si simplemente quieres combinar múltiples flujos.
- **`combineLatestWith`**: ideal si quieres combinar los últimos valores de otros flujos mientras transformas o procesas el flujo principal

## ⚠️ Notas.

### No se emite hasta que los valores iniciales estén disponibles.

No se emiten resultados hasta que todos los Observable hayan emitido al menos un valor.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // No se emite ningún valorObservable
).subscribe(console.log);
// Sin salida (porqueNEVERSin salida (porque)
```

Esto puede resolverse proporcionando un valor inicial con `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Salida: [0, null] → [1, null] → [2, null]
```

### Cuidado con las reemisiones frecuentes.

Si algún flujo emite valores con frecuencia, el resultado también se reemitirá con frecuencia.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msCorriente emitida cada
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$cada vez que se emite un valorfast$se combina con el último valor de
// → El rendimiento requiere atención
```

### Tratamiento de errores.

Si se produce un error en cualquier Observable, todo el proceso finaliza con un error.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Se producen errores')).pipe(
      catchError((err: unknown) => of('Recuperación'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Salida: [0, 'Recuperación'] → [1, 'Recuperación']
```

## 📚 Operadores relacionados.

- **[combineLatest](/es/guide/creation-functions/combination/combineLatest)** - versión de Creation Function.
- **[withLatestFrom](/es/guide/operators/combinación/withLatestFrom)** - Activado sólo por la corriente principal.
- **[zipWith](/es/guide/operators/combinación/zipWith)** - Empareja los valores correspondientes
