---
description: "combineLatestAll es un operador que toma un Observable de orden superior (Observable de Observable) y combina el último valor de cada uno una vez que todos los Observable internos se han disparado al menos una vez."
---

# combineLatestAll - combina los últimos valores del Observable interno

El operador combineLatestAll toma un **Observable de orden superior** (Observable of Observables),
**Una vez que todos los Observable internos se han disparado al menos una vez**, los **Últimos Valores** de cada uno se combinan y se muestran como un **Array**.

## 🔰 Sintaxis básica y uso

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Dos internosObservableconHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Todos los internosObservabletienen al menos1dispara una vez, combina los últimos valores
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Salida:
// [1, 3, 0] ← Cuando todos han disparado al menos1Cuando todos han disparado al menos una vez (después de2segundos después)
// [2, 3, 0] ← 1Cuando el segundoObservablese dispara (después de 2 segundos), el segundo2dispara (después de3segundos después)
// [2, 3, 1] ← 3Cuando el segundoObservablese dispara (después de 2 segundos), el segundo1dispara (después de4segundos después)
```

- Recopilar Observables internos cuando Observable de orden superior está **completo**.
- **Iniciar la combinación cuando todos los Observable Internos hayan disparado** al menos una vez.
- Siempre que cualquier Observable Interno emita un valor, **combinar** todos los últimos valores y **emitir**.

[🌐 Documentación oficial de RxJS - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Patrón de utilización típico.

- **Combinar los últimos resultados de múltiples llamadas a la API**.
- **Sincronizar los últimos valores de múltiples entradas de formularios**
- **Integrar múltiples fuentes de datos en tiempo real**

## 🧠 Ejemplos prácticos de código

Ejemplo de combinación de los últimos resultados de múltiples llamadas a la API

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Dos simulacrosAPICrear llamada (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Información del usuario (1Completada en segundos,3actualizada una vez)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Usuario: User${n}`)
  ),
  // API 2: Número de notificaciones (0.5Completada en segundos,4actualizada una vez)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notificaciones: ${n}Número de notificaciones`)
  ),
  // API 3: Estado2Completada en segundos,2actualizada una vez)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Estado: Desconectado' : 'Estado: En línea')
  )
);

// TodosAPIÚltimo valor combinado de las llamadas
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Último estado:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Se ejecutan tres llamadas a la API en paralelo.
- **Después de que todas se hayan disparado al menos una vez**, se muestran los resultados combinados.
- Cada vez que se actualiza una API, se muestra la **última combinación**.

## 🔄 Función de creación relacionada

combineLatestAll` se utiliza principalmente para aplanar Observable de orden superior,
Utilice la **Creation Function** `combineLatest` para combinaciones normales de varios Observables.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionEdición (uso más general)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Ver [Capítulo 3 Funciones de Creación - combineLatest](/es/guide/creation-functions/combination/combineLatest).

## 🔄 Operadores relacionados.

| Operador. | Descripción. |
|---|---|
| [mergeAll](./mergeAll) | Suscribe todos los Observable internos en paralelo. |
| [concatAll](./concatAll) | Suscribirse a Observable internos en secuencia. |
| [switchAll](./switchAll) | Cambiar a un nuevo Observable interno. |
| [zipAll](./zipAll) | Empareja los valores de cada Observable interno en el orden correspondiente |

## ⚠️ Notas.

### Debe completarse el Observable de orden superior.

combineLatestAll` espera la colección de Observables internos **hasta** que el Observable de orden superior (Observable externo) esté **completado**.

#### ❌ El Observable de orden superior no se completa, por lo que no se emite nada.

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // No se emite nada
```

#### ✅ Take to complete

```ts
interval(1000).pipe(
  take(3), // 3Se completa en una sesión
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Todos los Observable internos deben dispararse al menos una vez

No se emite ningún valor hasta que todos los Observable internos se hayan disparado **al menos una vez**.

```ts
// 1Si alguno de los internosObservableNo se emite nada si hay
of(
  of(1, 2, 3),
  NEVER // Nunca se dispara para siempre
).pipe(
  combineLatestAll()
).subscribe(console.log); // No se emite nada
```

### Uso de memoria.

Tenga en cuenta el uso de memoria si hay muchos Observable internos, ya que los **últimos valores de todos los Observable internos se mantienen en memoria**.
