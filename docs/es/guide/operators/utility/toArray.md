---
description: toArray es un operador de utilidad de RxJS que combina todos los valores emitidos hasta que Observable se completa en un solo array. Esto es ideal para situaciones donde desea tratar todo el stream como un array, como procesamiento por lotes, visualización de UI después de adquisición por lotes y procesamiento agregado. Debido a que acumula valores hasta completarse, no se puede usar con streams infinitos.
---

# toArray - Convertir Valores a Array

El operador `toArray` es un operador que **combina todos los valores emitidos por Observable hasta completarse en un solo array**.
Esto es útil para procesamiento por lotes, visualización de UI después de recuperación por lotes, agregación, etc.


## 🔰 Sintaxis Básica y Operación

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Salida:
// [1, 2, 3]
```

Todos los valores se combinan en un solo array, que se emite al completarse el Observable.

[🌐 Documentación Oficial de RxJS - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Ejemplo de Uso Típico

Esto se puede usar en situaciones donde desea procesar múltiples resultados asincrónicos a la vez o emitirlos a la UI en un lote.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Recibir todos al completarse:', result);
  });

// Salida:
// Recibir todos al completarse: [0, 1, 2, 3, 4]
```


## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Área de visualización de salida
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>Ejemplo de toArray:</h3>';
document.body.appendChild(toArrayOutput);

// Área de visualización de valores individuales
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Valores Individuales:</h4>';
toArrayOutput.appendChild(individualValues);

// Área de visualización de resultado de array
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Resultado de Array:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Suscribirse a valores individuales
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valor: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Suscribirse al mismo stream como array
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Array resultante: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Mostrar elementos del array individualmente
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Resumen

- `toArray` **emite un array de todos los valores al completarse**
- Ideal para situaciones donde desea manejar todo el stream de forma agregada
- Combinado con `concatMap`, `delay`, etc., se puede usar para **procesamiento secuencial asíncrono por lotes**
