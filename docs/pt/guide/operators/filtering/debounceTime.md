---
description: O operador debounceTime emite o último valor quando nenhum novo valor foi recebido por um tempo especificado após emitir eventos consecutivos. É ideal para otimizar entradas frequentes como digitação em caixa de pesquisa ou eventos de redimensionamento de janela.
titleTemplate: ':title'
---

# debounceTime - Controlar Disparo de Evento e Emitir Valor Após Esperar um Certo Tempo

O operador `debounceTime` emite o último valor após um valor ter sido emitido no stream se nenhum novo valor tiver sido emitido pelo tempo especificado.
É muito comumente usado em situações onde eventos frequentes precisam ser suprimidos, como caixas de pesquisa de entrada do usuário.

## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Se nenhuma entrada adicional for recebida dentro de 300ms após um evento de entrada ocorrer, o valor é emitido.
- Isto tem o efeito de consolidar eventos que ocorrem consecutivamente em um curto período de tempo.

[🌐 Documentação Oficial RxJS - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Padrões de Uso Típicos

- Enviar requisição após o usuário terminar de digitar na caixa de pesquisa
- Obter tamanho final para evento de redimensionamento de janela
- Obter posição final para evento de rolagem

## 🧠 Exemplo de Código Prático (com UI)

Quando um caractere é digitado na caixa de pesquisa, uma mensagem de início de pesquisa é exibida quando a entrada para por 300 ms.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Criar área de saída
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Digite palavra de pesquisa';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Stream de entrada
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Iniciou pesquisa por "${value}"`;
});
```

- Nenhuma resposta imediata enquanto estiver digitando
- Vai parar de digitar e iniciar pesquisa com o último valor de entrada 300ms depois
