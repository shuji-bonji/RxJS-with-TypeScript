---
description: mergeScan es un operador de transformación de RxJS que realiza procesamiento acumulativo asíncrono, combinando el comportamiento de scan y mergeMap. Es ideal para situaciones que requieren acumulación con procesamiento asíncrono, como agregación acumulativa de respuestas API, ejecución de la siguiente solicitud basada en resultados previos y obtención acumulativa de datos de múltiples páginas en paginación. También permite control del número de ejecuciones simultáneas mediante el parámetro concurrent.
---

# mergeScan - Acumulación con procesamiento asíncrono

El operador `mergeScan` ejecuta **procesamiento acumulativo asíncrono** para cada valor del stream.
Funciona como una combinación de `scan` y `mergeMap`, manteniendo un valor acumulativo mientras transforma cada valor en un nuevo Observable y utiliza ese resultado en el siguiente procesamiento acumulativo.

## 🔰 Sintaxis básica y uso

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take,  } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // Procesamiento asíncrono para cada valor (aquí se devuelve inmediatamente)
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// Salida: 0, 1, 3, 6, 10
```

- `acc` es el valor acumulativo, `curr` es el valor actual.
- La función acumulativa debe **devolver un Observable**.
- Los resultados del procesamiento de cada valor se van acumulando.

[🌐 Documentación oficial de RxJS - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Patrones de uso típicos

- Acumular y agregar respuestas API
- Ejecutar la siguiente solicitud API basándose en el resultado anterior
- Procesamiento acumulativo asíncrono de datos en tiempo real
- Obtención acumulativa de datos de múltiples páginas mediante paginación

## 📊 Diferencia con scan

| Operador | Valor de retorno de función acumulativa | Caso de uso |
|--------------|------------------|--------------|
| `scan` | Devuelve valor directamente | Procesamiento acumulativo síncrono |
| `mergeScan` | Devuelve Observable | Procesamiento acumulativo asíncrono |

```ts
// scan - procesamiento síncrono
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - procesamiento asíncrono
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 Ejemplo de código práctico (obtención acumulativa de API)

Ejemplo que añade nuevos datos a los resultados previos cada vez que se hace clic en un botón.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Obtener datos';
document.body.appendChild(button);

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// API simulada (devuelve datos con retardo)
const fetchData = (page: number) => {
  return of(`Datos${page}`).pipe(delay(500));
};

// Obtención acumulativa con evento de clic
fromEvent(button, 'click').pipe(
  take(5), // máximo 5 veces
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`Obteniendo página ${page}...`);

    // Añadir nuevos datos a los datos acumulados previos
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>Datos obtenidos:</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- Obtiene datos de forma asíncrona con cada clic.
- Añade nuevos datos a los resultados previos (`accumulated`).
- **Los resultados acumulativos se actualizan en tiempo real**.

## 🎯 Ejemplo práctico: procesamiento acumulativo con control de concurrencia

`mergeScan` tiene un parámetro `concurrent` que permite controlar el número de ejecuciones simultáneas.

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take, delay } from 'rxjs';

interface RequestLog {
  total: number;
  logs: string[];
}

interval(200).pipe(
  take(10),
  mergeScan((acc, curr) => {
    const timestamp = new Date().toLocaleTimeString();
    console.log(`Inicio de solicitud ${curr}: ${timestamp}`);

    // Cada solicitud tarda 1 segundo
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `Solicitud ${curr} completada: ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // 2 ejecuciones simultáneas
).subscribe((result) => {
  console.log(`Acumulado: ${result.total} elementos`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- Con `concurrent: 2`, ejecuta hasta 2 solicitudes simultáneamente.
- La tercera solicitud en adelante espera hasta que se complete la solicitud anterior.

## ⚠️ Puntos de atención

### 1. Manejo de errores

Si ocurre un error dentro de la función acumulativa, todo el stream se detiene.

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError(err => {
        console.error('Error ocurrido:', err);
        // Mantener valor acumulativo y continuar
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. Gestión de memoria

Tener cuidado de que el valor acumulativo no crezca demasiado

```ts
// Mal ejemplo: acumulación ilimitada
mergeScan((acc, curr) => of([...acc, curr]), [])

// Buen ejemplo: mantener solo los últimos N elementos
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // solo los últimos 100 elementos
}, [])
```

### 3. Usar scan si el procesamiento es síncrono

Si no se requiere procesamiento asíncrono, usa el simple `scan`.

```ts
// mergeScan es innecesario
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scan es suficiente
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 Operadores relacionados

- [`scan`](./scan.md) - Procesamiento acumulativo síncrono
- [`reduce`](./reduce.md) - Generar solo el valor acumulativo final al completar
- [`mergeMap`](./mergeMap.md) - Mapeo asíncrono (sin acumulación)
- [`expand`](./expand.md) - Procesamiento de expansión recursiva
