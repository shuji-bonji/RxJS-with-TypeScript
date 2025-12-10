---
description: Explica casos de uso prácticos de operadores de transformación de RxJS (map, mergeMap, switchMap, concatMap, scan, buffer, etc.). Procesamiento de entrada de usuario, formateo de respuestas API, solicitudes anidadas, agregación de datos, división de stream y procesamiento por lotes - introduce patrones prácticos para procesar y transformar datos flexiblemente con ejemplos de código TypeScript. Aprende patrones de transformación frecuentemente usados en el trabajo real.
---

# Patrones de transformación práctica

Los operadores de transformación son uno de los grupos de operadores más frecuentemente usados en RxJS.
Desempeñan un papel indispensable para procesar y transformar datos flexiblemente en programación reactiva.

Esta sección organiza patrones de uso de operadores de transformación mientras introduce ejemplos prácticos típicos.


## 💬 Patrones de uso típicos

| Patrón | Operadores representativos | Descripción |
|:---|:---|:---|
| Transformación simple de valores | `map` | Aplicar función de transformación a cada valor |
| Procesamiento acumulativo/agregación | `scan`, `reduce` | Acumular valores secuencialmente |
| Procesamiento asíncrono anidado | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Generar y combinar Observables |
| Procesamiento por lotes/agrupación | `bufferTime`, `bufferCount`, `windowTime` | Procesar agrupadamente, gestión dividida |
| Extracción de propiedades | `pluck` | Extraer campo específico de objeto |


## Validación y transformación de entrada de usuario

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Campo de entrada
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// Función de validación de dirección de correo
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Procesamiento de entrada
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Por favor ingrese dirección de correo',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Por favor ingrese dirección de correo válida',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'La dirección de correo es válida',
        value: email,
      };
    })
  )
  .subscribe((result) => {
    if (result.isValid) {
      emailStatus.textContent = '✓ ' + result.message;
      emailStatus.className = 'valid';
    } else {
      emailStatus.textContent = '✗ ' + result.message;
      emailStatus.className = 'invalid';
    }
  });
```

## Transformación y agregación de array de objetos

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Datos de ventas
const sales = [
  { product: 'Laptop', price: 120000, quantity: 3 },
  { product: 'Tablet', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Mouse', price: 3500, quantity: 12 },
  { product: 'Keyboard', price: 6500, quantity: 8 },
];

// Transformación y agregación de datos
from(sales)
  .pipe(
    // Calcular monto total por producto
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Añadir precio con impuestos
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Convertir de nuevo a array
    toArray(),
    // Calcular monto total
    map((items) => {
      const grandTotal = items.reduce((sum, item) => sum + item.total, 0);
      const grandTotalWithTax = items.reduce(
        (sum, item) => sum + item.totalWithTax,
        0
      );
      return {
        items,
        grandTotal,
        grandTotalWithTax,
      };
    })
  )
  .subscribe((result) => {
    console.log('Detalles de productos:', result.items);
    console.log('Monto total (sin impuestos):', result.grandTotal);
    console.log('Monto total (con impuestos):', result.grandTotalWithTax);
  });
// Salida:
// Detalles de productos: (5) [{…}, {…}, {…}, {…}, {…}]
// Monto total (sin impuestos): 1109000
// Monto total (con impuestos): 1219900
```

## Normalización de datos JSON

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs';

const resultBox = document.createElement('div');
resultBox.id = 'normalized-results';
document.body.appendChild(resultBox);

ajax
  .getJSON<any[]>('https://jsonplaceholder.typicode.com/users')
  .pipe(
    map((users) => {
      // Convertir a objeto con ID como clave
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Aplanar objeto anidado
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Eliminar anidamiento innecesario
          company: undefined,
          address: undefined,
        };
        userIds.push(user.id);
      });

      return {
        entities: normalizedUsers,
        ids: userIds,
      };
    })
  )
  .subscribe((result) => {
    const title = document.createElement('h3');
    title.textContent = 'Datos de usuario normalizados';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Nombre de usuario: @${user.username}<br>
      Email: ${user.email}<br>
      Empresa: ${user.companyName}<br>
      Dirección: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Acceso rápido a usuario con ID específico posible
    console.log('Usuario ID 3:', result.entities[3]);
  });

```

## Combinación de múltiples transformaciones

En aplicaciones reales, es común usar múltiples operadores de transformación en combinación.

```ts
import { fromEvent, timer } from 'rxjs';
import {
  switchMap,
  map,
  tap,
  debounceTime,
  takeUntil,
  distinctUntilChanged,
} from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
  company: {
    name: string;
  };
};

// Entrada de búsqueda
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Procesamiento de búsqueda
fromEvent(searchInput, 'input')
  .pipe(
    // Obtener valor de entrada
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Esperar 300ms
    debounceTime(300),
    // Ignorar si es el mismo valor
    distinctUntilChanged(),
    // Mostrar indicador de carga
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // Solicitud API (cancelar solicitud anterior)
    switchMap((term) => {
      // Sin resultados si entrada vacía
      if (term === '') {
        return [];
      }

      // Procesamiento de timeout (5 segundos)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('Timeout de respuesta API')),
        map(() => [{ error: 'Timeout' }])
      );

      // Llamada API
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Procesar resultados
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Completar antes del timeout
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Finalizar carga
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">Usuario no encontrado</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Empresa: ${user.company}</p>
          </div>
        `
          )
          .join('');
      }
    } else {
      resultsContainer.innerHTML = `<div class="error">⚠️ ${result}</div>`;
    }
  });

```

## 🧠 Resumen

- Transformación simple: `map`
- Si se maneja procesamiento asíncrono: `mergeMap`, `switchMap`, `concatMap`, `exhaustMap`
- Procesamiento por lotes: `bufferTime`, `bufferCount`
- Extracción de propiedades: `pluck`
- En aplicaciones reales **es normal combinar estos**

Al dominar los operadores de transformación, podrás manejar flujos de datos asíncronos complejos
de manera intuitiva y declarativa!
