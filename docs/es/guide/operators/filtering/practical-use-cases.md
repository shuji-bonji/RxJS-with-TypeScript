---
description: Los casos de uso prácticos de los operadores de filtrado de RxJS (debounceTime, throttleTime, distinctUntilChanged, filter, etc.) se explican. Aprende patrones prácticos para extraer solo los valores que necesitas de los flujos, como búsqueda en tiempo real, desplazamiento infinito, control de eventos de alta frecuencia, deduplicación, etc., con ejemplos de código TypeScript. Aprenderás técnicas de implementación útiles para el manejo de eventos de UI y optimización de rendimiento.
---

# Casos de Uso Prácticos

## Filtrado de Búsqueda en Tiempo Real de Entrada de Usuario

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Construir UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Ingrese término de búsqueda (3+ caracteres)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Flujo de eventos
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Iniciando búsqueda de "${searchTerm}"...`;
  });

```

- **Procesa solo entrada confirmada** a intervalos de 300ms.
- **Las búsquedas se realizan solo cuando se ingresan 3 o más caracteres**.
- **Las entradas consecutivas de la misma palabra** se ignoran.


## Simulación de Desplazamiento Infinito

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Construir UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Agregar datos iniciales
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Elemento ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Flujo de evento de desplazamiento
fromEvent(scrollArea, 'scroll')
  .pipe(
    throttleTime(200),
    map(() => ({
      scrollTop: scrollArea.scrollTop,
      scrollHeight: scrollArea.scrollHeight,
      clientHeight: scrollArea.clientHeight,
    })),
    map(
      ({ scrollTop, scrollHeight, clientHeight }) =>
        (scrollTop + clientHeight) / scrollHeight
    ),
    distinctUntilChanged(),
    filter((ratio) => ratio > 0.8),
    scan((page) => page + 1, 1),
    filter((page) => page <= 5)
  )
  .subscribe((page) => {
    addItems(page);
  });

```

- Cuando la posición de desplazamiento alcanza **80% o más**, se cargan los siguientes elementos.
- **Carga automática hasta 5 páginas**.
- **Los eventos de desplazamiento** se limitan **cada 200ms**.


## Resumen de Cómo Elegir Operadores de Filtrado

| Lo Que Deseas Hacer | Operador | Descripción |
|:---|:---|:---|
| Solo pasar datos que coincidan con la condición | `filter` | Filtrado más básico |
| Obtener solo los primeros elementos | `take`, `first` | Limitar número de elementos adquiridos |
| Esperar hasta que se confirme la entrada | `debounceTime` | Ideal para entrada de formulario |
| Procesar solo a intervalos fijos | `throttleTime` | Aplicar a desplazamiento, redimensionar, etc. |
| Ignorar valores consecutivos iguales | `distinctUntilChanged` | Prevenir reprocesamiento desperdiciado de datos idénticos |


## Resumen

- Los operadores de filtrado son esenciales para controlar flujos de datos.
- No solo son poderosos cuando se usan solos, sino aún más cuando se **combinan**.
- Conducen directamente a **mejora de eficiencia y rendimiento** en aplicaciones impulsadas por eventos y desarrollo de UI.
