---
description: O operador throttleTime reduz eficientemente eventos de alta frequência ao permitir que apenas o primeiro valor passe dentro de um intervalo de tempo especificado e ignorar valores subsequentes. É ideal para otimização de eventos em tempo real como rolagem ou movimento do mouse.
titleTemplate: ':title'
---

# throttleTime - Primeiro valor depois limite

O operador `throttleTime` passa o primeiro valor emitido e ignora valores subsequentes emitidos dentro de um intervalo de tempo especificado.
Ele não emite o último valor em intervalos regulares, mas sim **apenas passa o primeiro valor que recebe e ignora valores subsequentes durante esse período**.

Isto é útil para reduzir streams que disparam com frequência, como eventos de rolagem e eventos de movimento do mouse.


## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Clicado!'));

```

- Recebe apenas o primeiro evento de clique a cada 2 segundos e ignora cliques subsequentes.

[🌐 Documentação Oficial RxJS - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)

> [!WARNING] Atenção em código de produção
> O exemplo acima omite o cancelamento da inscrição de `fromEvent` para simplificar a explicação. Em código real, gerencie explicitamente o ciclo de vida com `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Detalhes: [Superar dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Padrões de Uso Típicos

- Otimização de tratamento de eventos para rolagem e movimento do mouse
- Prevenção de múltiplas submissões devido a pressionamentos consecutivos de botão
- Redução de stream de dados em tempo real


## 🧠 Exemplo de Código Prático (com UI)

Quando o mouse é movido, informações de posição são exibidas a cada 100 milissegundos.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Criar área de saída
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Por favor, mova seu mouse dentro desta área';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Evento de movimento do mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Posição do mouse: X=${position.x}, Y=${position.y}`;
});
```

- Limita eventos de movimento do mouse disparados com frequência a cada 100ms e exibe apenas a posição mais recente.
