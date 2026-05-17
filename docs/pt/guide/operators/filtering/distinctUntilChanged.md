---
description: O operador distinctUntilChanged permite o processamento eficiente de dados ao ignorar valores consecutivos que são iguais ao anterior e emitir apenas os valores que mudaram.
titleTemplate: ':title | RxJS'
---

# distinctUntilChanged - Sem Duplicados

O operador `distinctUntilChanged` remove duplicatas quando o mesmo valor é emitido consecutivamente, e emite apenas o novo valor se ele diferir do valor anterior.


## 🔰 Sintaxe Básica e Uso

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Saída: 1, 2, 3, 1, 2, 3
```

- Se o valor for igual ao anterior, ele é ignorado.
- Isto não é um processo em lote como `Array.prototype.filter`, mas sim uma **decisão sequencial**.

[🌐 Documentação Oficial RxJS - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)

> [!WARNING] Atenção em código de produção
> O exemplo acima omite o cancelamento da inscrição de `fromEvent` para simplificar a explicação. Em código real, gerencie explicitamente o ciclo de vida com `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Detalhes: [Superar dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Padrões de Uso Típicos

- Detecção de entrada em formulários para evitar requisições desperdiçadas para valores de entrada consecutivos iguais
- Detectando mudanças em sensores e streams de eventos
- Evitar redesenhos desnecessários de UI no gerenciamento de estado


## 🧠 Exemplo de Código Prático (com UI)

Simulação de envio de uma requisição de API em uma caixa de pesquisa **apenas se a string digitada for diferente da anterior**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Criar área de saída
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Digite palavras-chave de pesquisa';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Stream de entrada
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Executar com valor de pesquisa: ${keyword}`;
  });

```

- Se o texto de entrada não mudar, não será requisitado.
- Isto pode ser usado para processamento de pesquisa eficiente e otimização de comunicação com API.
